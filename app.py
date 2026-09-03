"""
ROS 2 Command Center - a TUI for inspecting the ROS 2 graph.

Tabs: Topics (type / publishers / subscribers / QoS / Hz / echo),
Nodes (pubs / subs / services / clients) and Services (type / servers).
"""
from collections import deque
from datetime import datetime
import os
import shlex
import shutil
import signal
import subprocess
import time

import rclpy
import yaml
from rcl_interfaces.srv import GetParameters, ListParameters, SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message
from rich.console import Group
from rich.markup import escape
from rich.table import Table
from rich.text import Text
from rich.tree import Tree
from rosidl_runtime_py import message_to_yaml, set_message_fields
from rosidl_runtime_py.utilities import get_action, get_message, get_service
from textual.app import App, ComposeResult
from textual.containers import Horizontal, Vertical, VerticalScroll
from textual.screen import ModalScreen
from textual.widgets import (
    Button, DataTable, Input, Label, Select, Sparkline, Static, TabbedContent, TabPane,
    TextArea,
)

try:
    from diagnostic_msgs.msg import DiagnosticArray
    _DIAGNOSTICS_AVAILABLE = True
except Exception:  # noqa: BLE001
    _DIAGNOSTICS_AVAILABLE = False

try:
    from lifecycle_msgs.msg import Transition as LifecycleTransition
    from lifecycle_msgs.srv import ChangeState, GetState

    _LIFECYCLE_AVAILABLE = True
except Exception:  # noqa: BLE001
    _LIFECYCLE_AVAILABLE = False

try:
    from rclpy.action.graph import (
        get_action_client_names_and_types_by_node,
        get_action_names_and_types,
        get_action_server_names_and_types_by_node,
    )
    _ACTIONS_AVAILABLE = True
except Exception:  # noqa: BLE001
    _ACTIONS_AVAILABLE = False

try:
    from rclpy.action import ActionClient

    _ACTION_CLIENT_AVAILABLE = True
except Exception:  # noqa: BLE001
    _ACTION_CLIENT_AVAILABLE = False

try:
    from tf2_msgs.msg import TFMessage
    _TF_AVAILABLE = True
except Exception:  # noqa: BLE001
    _TF_AVAILABLE = False

Log = get_message("rcl_interfaces/msg/Log")
LOG_LEVELS = {
    10: ("DEBUG", "#8e8e93"),
    20: ("INFO", "#00b7ff"),
    30: ("WARN", "#ffb000"),
    40: ("ERROR", "#ff4f55"),
    50: ("FATAL", "#ff4f55"),
}
DIAGNOSTIC_LEVELS = {
    0: ("OK", "#ffffff"),
    1: ("WARN", "#ffb000"),
    2: ("ERROR", "#ff4f55"),
    3: ("STALE", "#8e8e93"),
} if _DIAGNOSTICS_AVAILABLE else {}

LIFECYCLE_TRANSITIONS = {}
if _LIFECYCLE_AVAILABLE:
    LIFECYCLE_TRANSITIONS = {
        "configure": LifecycleTransition.TRANSITION_CONFIGURE,
        "cleanup": LifecycleTransition.TRANSITION_CLEANUP,
        "activate": LifecycleTransition.TRANSITION_ACTIVATE,
        "deactivate": LifecycleTransition.TRANSITION_DEACTIVATE,
        "shutdown": LifecycleTransition.TRANSITION_ACTIVE_SHUTDOWN,
    }

REFRESH_SECONDS = 2.0
SPIN_SECONDS = 0.02
HZ_WINDOW = 50
ECHO_MAX_LINES = 40
LOG_MAX_LINES = 200

TAB_IDS = (
    "topics", "nodes", "services", "graph", "tf", "actions",
    "logs", "diagnostics", "operations",
)


def full_name(node_name: str, namespace: str) -> str:
    """Join a node name and namespace into a fully qualified name."""
    if namespace in ("", "/"):
        return f"/{node_name}"
    return f"{namespace.rstrip('/')}/{node_name}"


def qos_summary(qos: QoSProfile) -> str:
    """Short QoS description, e.g. 'RELIABLE VOLATILE d=10'."""
    rel = ReliabilityPolicy(qos.reliability).name
    dur = DurabilityPolicy(qos.durability).name
    return f"{rel} {dur} d={qos.depth}"


def yaml_inline(value) -> str:
    return yaml.safe_dump(value, default_flow_style=True).replace("\n...", "").strip()


def interface_definition_lines(type_name: str, kind: str = "msg") -> list[str]:
    """Return rich-text lines describing a msg/srv/action interface."""
    try:
        if kind == "msg":
            cls = get_message(type_name)
            sections = [("fields", cls)]
        elif kind == "srv":
            cls = get_service(type_name)
            sections = [("request", cls.Request), ("response", cls.Response)]
        elif kind == "action":
            cls = get_action(type_name)
            sections = [
                ("goal", cls.Goal),
                ("result", cls.Result),
                ("feedback", cls.Feedback),
            ]
        else:
            return []
    except Exception as exc:
        return [f"  [red]cannot load {kind} {type_name}: {exc}[/]"]

    out = []
    for title, section_cls in sections:
        out.append(f"  [b]{title}[/]")
        fields = section_cls.get_fields_and_field_types()
        if not fields:
            out.append("    [dim]no fields[/]")
            continue
        for field_name, field_type in fields.items():
            out.append(f"    {escape(field_name)} [dim]{escape(field_type)}[/]")
    return out


def _first_numeric_field(obj, path=()):
    """Return (value, path) for the first numeric scalar in a message, or None."""
    if isinstance(obj, bool):
        return None
    if isinstance(obj, (int, float)):
        return obj, path
    if hasattr(obj, "get_fields_and_field_types"):
        for name in obj.get_fields_and_field_types():
            value = getattr(obj, name)
            result = _first_numeric_field(value, path + (name,))
            if result is not None:
                return result
    if isinstance(obj, (list, tuple)) and obj:
        result = _first_numeric_field(obj[0], path + (0,))
        if result is not None:
            return result
    return None


def topic_name_cell(name: str) -> Text:
    namespace, _, leaf = name.rpartition("/")
    text = Text(leaf or "/", style="bold #f5f5f7", overflow="ellipsis", no_wrap=True)
    if namespace:
        text.append("\n")
        text.append(namespace, style="#636366")
    return text


def topic_type_cell(types: list[str]) -> Text:
    if not types:
        return Text("Unknown", style="#636366")
    package, _, interface = types[0].rpartition("/")
    text = Text(interface, style="bold #f5f5f7", overflow="ellipsis", no_wrap=True)
    text.append("\n")
    suffix = f" +{len(types) - 1}" if len(types) > 1 else ""
    text.append(f"{package.split('/')[0]}{suffix}", style="#636366")
    return text


def topic_activity_cell(publishers: int, subscribers: int) -> Text:
    if publishers and subscribers:
        state, color = "● CONN", "#ffffff"
    elif publishers:
        state, color = "● PUB", "#00b7ff"
    elif subscribers:
        state, color = "● WAIT", "#ffb000"
    else:
        state, color = "○ IDLE", "#636366"
    text = Text(state, style=f"bold {color}")
    text.append("\n")
    text.append(f"pub:{publishers} sub:{subscribers}", style="#8e8e93")
    return text


class CommandForm(ModalScreen[dict | None]):
    CSS = """
    CommandForm {
        align: center middle;
        background: #000000 65%;
    }
    #command_dialog {
        width: 72;
        height: auto;
        max-height: 90%;
        padding: 1 2;
        background: #121214;
        border: round #3a3a3c;
    }
    #command_title {
        height: 2;
        color: #ffffff;
        text-style: bold;
    }
    CommandForm Input {
        width: 100%;
        margin: 0 0 1 0;
    }
    CommandForm TextArea {
        height: 10;
        margin-bottom: 1;
        background: #1c1c1e;
        border: round #2c2c2e;
    }
    #command_buttons {
        height: 3;
        align-horizontal: right;
    }
    #command_buttons Button {
        margin-left: 1;
    }
    """

    def __init__(self, title: str, fields: list[tuple[str, str]], payload: str = "") -> None:
        super().__init__()
        self.form_title = title
        self.fields = fields
        self.payload = payload

    def compose(self) -> ComposeResult:
        with Vertical(id="command_dialog"):
            yield Static(self.form_title, id="command_title")
            for index, (label, value) in enumerate(self.fields):
                yield Input(value=value, placeholder=label, id=f"command_field_{index}")
            yield Label("YAML / arguments")
            yield TextArea(self.payload, id="command_payload", show_line_numbers=False)
            with Horizontal(id="command_buttons"):
                yield Button("Cancel", id="command_cancel", flat=True)
                yield Button("Confirm", id="command_confirm", variant="success")

    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == "command_cancel":
            self.dismiss(None)
            return
        values = [
            self.query_one(f"#command_field_{index}", Input).value
            for index in range(len(self.fields))
        ]
        payload = self.query_one("#command_payload", TextArea).text
        self.dismiss({"fields": values, "payload": payload})


class CommandPalette(ModalScreen[str | None]):
    """A modal fuzzy finder over topics, nodes, services and actions."""

    CSS = """
    CommandPalette {
        align: center middle;
        background: #000000 65%;
    }
    #palette_dialog {
        width: 80;
        height: auto;
        max-height: 80%;
        padding: 1 2;
        background: #121214;
        border: round #3a3a3c;
    }
    #palette_filter {
        width: 100%;
        margin: 0 0 1 0;
    }
    #palette_table {
        width: 100%;
        height: 18;
        background: #121214;
        border: none;
        scrollbar-size: 1 1;
    }
    """

    BINDINGS = [("escape", "dismiss_palette")]

    def __init__(self, candidates: list[tuple[str, str, str]]) -> None:
        super().__init__()
        self.all_candidates = candidates

    def compose(self) -> ComposeResult:
        with Vertical(id="palette_dialog"):
            yield Input(placeholder="type to filter...", id="palette_filter")
            table = DataTable(id="palette_table", cursor_type="row")
            table.add_column("Kind", width=8)
            table.add_column("Name", width=45)
            table.add_column("Detail", width=None)
            table.zebra_stripes = False
            table.show_header = False
            yield table

    def on_mount(self) -> None:
        self.query_one("#palette_filter", Input).focus()
        self._populate("")

    def on_input_changed(self, event: Input.Changed) -> None:
        if event.input.id == "palette_filter":
            self._populate(event.input.value.lower())

    def on_input_submitted(self, event: Input.Submitted) -> None:
        if event.input.id == "palette_filter":
            table = self.query_one("#palette_table", DataTable)
            if table.row_count:
                table.action_select_cursor()

    def _populate(self, needle: str) -> None:
        table = self.query_one("#palette_table", DataTable)
        table.clear()
        for kind, name, detail in self.all_candidates:
            if needle in f"{kind} {name} {detail}".lower():
                table.add_row(kind, name, detail, key=f"{kind}:{name}")
        if table.row_count:
            table.move_cursor(row=0)

    def action_dismiss_palette(self) -> None:
        self.dismiss(None)

    def on_data_table_row_selected(self, event: DataTable.RowSelected) -> None:
        if event.row_key is not None:
            self.dismiss(event.row_key.value)


class GraphCanvas(VerticalScroll):
    can_focus = True


class CommandCenterApp(App):
    """A ROS 2 graph inspector: topics, nodes and services in one place."""

    TITLE = "ROS 2 Command Center"
    SUB_TITLE = "Live graph inspector"

    CSS = """
    Screen {
        background: #121214;
        color: #d7d7d7;
        padding: 0 1;
    }
    #shell {
        height: 1fr;
    }
    #masthead {
        height: 3;
        margin: 0 1;
        border-bottom: solid #2c2c2e;
    }
    #brand {
        width: 1fr;
        content-align: left middle;
        color: #eeeeee;
        text-style: bold;
    }
    #graph_status {
        width: auto;
        content-align: right middle;
        color: #ffffff;
    }
    TabbedContent {
        height: 1fr;
        margin: 0 1;
        background: #121214;
        border: none;
    }
    Tabs {
        height: 3;
        background: #121214;
        color: #666666;
    }
    Tab {
        background: #121214;
        color: #777777;
        padding: 0 2;
    }
    Tab:hover {
        color: #d7d7d7;
        background: #1c1c1e;
    }
    Tab.-active {
        color: #ffffff;
        background: #121214;
        text-style: bold;
        border-bottom: none;
    }
    TabPane {
        padding: 0;
        background: #121214;
    }
    ContentSwitcher {
        background: #121214;
    }
    Input {
        width: 60%;
        height: 2;
        margin: 0 1;
        padding: 0 1;
        background: #121214;
        color: #d7d7d7;
        border: none;
        border-bottom: solid #2c2c2e;
    }
    Input > .input--placeholder {
        color: #555555;
    }
    Input:focus {
        border-bottom: solid #ffffff;
    }
    .split {
        height: 1fr;
        margin-top: 1;
    }
    .panel {
        height: 1fr;
        background: #121214;
        border: round #2c2c2e;
    }
    .list_panel {
        width: 6fr;
        margin-right: 1;
    }
    .panel_title {
        height: 2;
        padding: 0 1;
        color: #aaaaaa;
        text-style: bold;
    }
    #table_topics {
        background: #121214;
        border: none;
    }
    .graph_panel {
        width: 4fr;
        margin-right: 1;
    }
    .legend_panel {
        width: 1fr;
        padding: 0;
    }
    #graph_canvas {
        height: 1fr;
        padding: 0 1;
        scrollbar-size: 1 1;
    }
    #graph_canvas:focus {
        background: #1c1c1e;
    }
    #graph_view, #graph_legend {
        width: 100%;
    }
    #graph_legend {
        padding: 0 1;
    }
    #topic_sparkline {
        display: none;
        height: 6;
        margin: 1 0 0 0;
    }
    #tf_view, #actions_view, #log_view, #diagnostics_view, #operations_view {
        width: 1fr;
        height: 1fr;
        padding: 0 1;
        scrollbar-size: 1 1;
    }
    .log_line {
        text-style: none;
    }
    .diag_line {
        text-style: none;
    }
    DataTable {
        width: 1fr;
        height: 1fr;
        background: #121214;
        color: #d7d7d7;
        border: none;
        scrollbar-size: 1 1;
    }
    DataTable > .datatable--header {
        background: #121214;
        color: #666666;
        text-style: bold;
    }
    DataTable > .datatable--odd-row {
        background: #1c1c1e;
    }
    DataTable > .datatable--cursor {
        background: #2c2c2e;
        color: #ffffff;
        text-style: bold;
    }
    .detail {
        width: 2fr;
        padding: 0;
        scrollbar-size: 1 1;
    }
    .detail > Static {
        width: 100%;
    }
    .detail_info {
        padding: 1;
    }
    #settings_scroll {
        padding: 1 2;
    }
    .settings_label {
        color: #8e8e93;
        margin-top: 1;
    }
    .settings_value {
        color: #e5e5ea;
    }
    #settings_scroll Select {
        width: 20;
        margin: 0 0 1 0;
    }
    #settings_scroll Input {
        width: 20;
        margin: 0 0 1 0;
    }
    #apply_settings {
        margin: 1 0;
        padding: 0 2;
        background: #2c2c2e;
        color: #ffffff;
        border: round #2c2c2e;
        text-style: bold;
    }
    #status_bar {
        height: 2;
        padding: 0 2;
        color: #555555;
        background: #121214;
    }
    """

    BINDINGS = [
        ("ctrl+p", "command_palette", "Search"),
        ("r", "refresh", "Refresh"),
        ("p", "toggle_pause", "Pause"),
        ("e", "toggle_echo", "Echo"),
        ("x", "hide", "Hide"),
        ("m", "edit_parameter", "Parameter"),
        ("c", "call_service", "Call"),
        ("u", "publish_topic", "Publish"),
        ("g", "send_action_goal", "Goal"),
        ("b", "record_bag", "Record"),
        ("l", "launch", "Launch"),
        ("s", "stop_processes", "Stop"),
        ("L", "lifecycle_transition", "Lifecycle"),
        ("slash", "focus_filter", "Filter"),
        ("1", "switch_tab('topics')", "Topics"),
        ("2", "switch_tab('nodes')", "Nodes"),
        ("3", "switch_tab('services')", "Services"),
        ("4", "switch_tab('graph')", "Graph"),
        ("5", "switch_tab('tf')", "TF"),
        ("6", "switch_tab('actions')", "Actions"),
        ("7", "switch_tab('logs')", "Logs"),
        ("8", "switch_tab('diagnostics')", "Diagnostics"),
        ("9", "switch_tab('operations')", "Operations"),
        ("0", "switch_tab('settings')", "Settings"),
        ("q", "quit", "Quit"),
    ]

    def compose(self) -> ComposeResult:
        with Vertical(id="shell"):
            with Horizontal(id="masthead"):
                yield Static("ros2-cmdctr  [dim]v0.2[/]", id="brand")
                yield Static("● connecting", id="graph_status")
            with TabbedContent(initial="topics"):
                for tab in TAB_IDS:
                    with TabPane(tab, id=tab):
                        yield Input(
                            placeholder=f"filter {tab} by name or type...",
                            id=f"filter_{tab}",
                        )
                        if tab == "graph":
                            with Horizontal(classes="split"):
                                with Vertical(classes="panel graph_panel"):
                                    yield Static("[ graph ]", markup=False, classes="panel_title")
                                    with GraphCanvas(id="graph_canvas"):
                                        yield Static("Discovering graph...", id="graph_view")
                                with Vertical(classes="panel legend_panel"):
                                    yield Static("[ legend ]", markup=False, classes="panel_title")
                                    yield Static(
                                        "[#ffffff]● active[/]\n"
                                        "[#ff4f55]● no publisher[/]\n"
                                        "[#ffb000]● no subscriber[/]\n\n"
                                        "[b]Flow[/]\n"
                                        "pub ─▶ topic ─▶ sub\n\n"
                                        "[dim]filter: topic, type, node[/]",
                                        id="graph_legend",
                                    )
                        elif tab == "actions":
                            with Horizontal(classes="split"):
                                with Vertical(classes="panel list_panel"):
                                    yield Static(
                                        "[ actions ]", markup=False, classes="panel_title"
                                    )
                                    yield DataTable(
                                        id="table_actions",
                                        cursor_foreground_priority="css",
                                    )
                                with VerticalScroll(
                                    classes="panel detail", id="detail_actions"
                                ):
                                    yield Static(
                                        "[ inspector ]", markup=False, classes="panel_title"
                                    )
                                    yield Static(
                                        "Select an action to inspect.",
                                        id="info_actions",
                                        classes="detail_info",
                                    )
                        elif tab in ("tf", "logs", "diagnostics", "operations"):
                            with Vertical(classes="panel"):
                                yield Static(f"[ {tab} ]", markup=False, classes="panel_title")
                                view_id = {
                                    "tf": "tf_view",
                                    "logs": "log_view",
                                    "diagnostics": "diagnostics_view",
                                    "operations": "operations_view",
                                }[tab]
                                yield VerticalScroll(
                                    Static("Waiting for data...", id=view_id),
                                    can_focus=True,
                                    id=f"{tab}_scroll",
                                )
                        else:
                            with Horizontal(classes="split"):
                                with Vertical(classes="panel list_panel"):
                                    yield Static(f"[ {tab} ]", markup=False, classes="panel_title")
                                    yield DataTable(
                                        id=f"table_{tab}",
                                        cursor_foreground_priority="css",
                                    )
                                with VerticalScroll(
                                    classes="panel detail", id=f"detail_{tab}"
                                ):
                                    yield Static(
                                        "[ inspector ]", markup=False, classes="panel_title"
                                    )
                                    yield Static(
                                        "Select an item to inspect.",
                                        id=f"info_{tab}",
                                        classes="detail_info",
                                    )
                                    if tab == "topics":
                                        yield Sparkline(
                                            data=[],
                                            id="topic_sparkline",
                                        )
                with TabPane("\u2699", id="settings"):
                    with VerticalScroll(can_focus=True, id="settings_scroll"):
                        yield Static("[ settings ]", markup=False, classes="panel_title")
                        yield Static("Theme", classes="settings_label")
                        yield Select(
                            [("Dark", "dark"), ("Light", "light")],
                            allow_blank=False,
                            value="dark",
                            id="theme_select",
                        )
                        yield Static("Refresh interval (seconds)", classes="settings_label")
                        yield Input(value=f"{REFRESH_SECONDS:g}", id="refresh_input")
                        yield Static("ROS_DOMAIN_ID", classes="settings_label")
                        yield Static(
                            f"[#8e8e93]{os.environ.get('ROS_DOMAIN_ID', '0')}[/]",
                            classes="settings_value",
                            id="domain_value",
                        )
                        yield Static("Graph style", classes="settings_label")
                        yield Select(
                            [("Tree", "tree"), ("Flow arrows", "arrows")],
                            allow_blank=False,
                            value="tree",
                            id="graph_style_select",
                        )
                        yield Button("Apply", id="apply_settings", variant="primary")

            yield Static("", id="status_bar")

    # ------------------------------------------------------------------ setup

    def on_mount(self) -> None:
        if not rclpy.ok():
            rclpy.init()
        self.node = Node("ros2_command_center")

        # Graph caches: name -> data
        self.topics: dict[str, dict] = {}
        self.nodes: dict[str, dict] = {}
        self.services: dict[str, dict] = {}
        self.selected = {tab: None for tab in TAB_IDS}
        self.hidden: set[str] = set()

        # Echo / Hz state
        self.echo_enabled = True
        self.echo_sub = None
        self.echo_topic = None
        self.echo_error = None
        self.last_msg = None
        self.msg_count = 0
        self.msg_times: deque[float] = deque(maxlen=HZ_WINDOW)
        self.msg_sizes: deque[tuple[float, int]] = deque(maxlen=HZ_WINDOW)

        # Sparkline state
        self.sparkline_path: list[str] = []
        self.sparkline_values: deque[float] = deque(maxlen=50)

        # Control state
        self.node_parameters: dict[str, dict] = {}
        self.parameter_requests: set[str] = set()
        self.node_lifecycle: dict[str, dict | None] = {}
        self.lifecycle_requests: set[str] = set()
        self.control_clients = []
        self.action_clients: list = []
        self.ephemeral_publishers = []
        self.processes: dict[str, subprocess.Popen] = {}
        self.operation_events: deque[str] = deque(maxlen=20)

        # Logs / diagnostics state
        self.log_lines: deque[Text] = deque(maxlen=LOG_MAX_LINES)
        self.diagnostics: dict[str, dict] = {}
        self.actions: dict[str, dict] = {}
        self.tf_edges: dict[tuple[str, str], dict] = {}
        self.log_sub = self.node.create_subscription(Log, "/rosout", self.on_rosout, 10)
        if _DIAGNOSTICS_AVAILABLE:
            self.diag_sub = self.node.create_subscription(
                DiagnosticArray, "/diagnostics", self.on_diagnostics, 10
            )
        else:
            self.diag_sub = None
        if _TF_AVAILABLE:
            tf_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
            static_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.tf_sub = self.node.create_subscription(
                TFMessage, "/tf", lambda msg: self.on_tf(msg, False), tf_qos
            )
            self.tf_static_sub = self.node.create_subscription(
                TFMessage, "/tf_static", lambda msg: self.on_tf(msg, True), static_qos
            )
        else:
            self.tf_sub = None
            self.tf_static_sub = None

        self.paused = False
        self.refresh_seconds = REFRESH_SECONDS
        self.graph_style = "tree"

        columns = {
            "topics": (("Topic", 17), ("Interface", 17), ("Activity", 16)),
            "nodes": (("Node", None), ("Pub", 5), ("Sub", 5), ("Srv", 5), ("Cli", 5)),
            "services": (("Service", None), ("Type", None), ("Servers", 9)),
            "actions": (("Action", 18), ("Type", 18), ("Servers", 7), ("Clients", 7)),
        }
        for tab, cols in columns.items():
            table = self.query_one(f"#table_{tab}", DataTable)
            table.cursor_type = "row"
            table.zebra_stripes = False
            table.show_header = False
            table.cell_padding = 1 if tab == "topics" else 0
            for label, width in cols:
                table.add_column(label, width=width, key=label.lower())

        self.refresh_graph()
        self.refresh_timer = self.set_interval(self.refresh_seconds, self.refresh_graph)
        self.set_interval(SPIN_SECONDS, self.spin_once)
        self.set_interval(0.5, self.update_topic_details)
        self.set_interval(0.3, self.update_logs)
        self.set_interval(0.5, self.update_diagnostics)
        self.set_interval(0.5, self.update_tf)
        self.query_one("#table_topics", DataTable).focus()

    def on_unmount(self) -> None:
        self.stop_echo()
        if hasattr(self, "node"):
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    # ---------------------------------------------------------------- actions

    def action_refresh(self) -> None:
        self.refresh_graph()

    def action_toggle_pause(self) -> None:
        self.paused = not self.paused
        if self.paused:
            self.refresh_timer.pause()
        else:
            self.refresh_timer.resume()
            self.refresh_graph()
        self.update_status()

    def action_toggle_echo(self) -> None:
        self.echo_enabled = not self.echo_enabled
        if self.echo_enabled:
            self.start_echo(self.selected["topics"])
        else:
            self.stop_echo()
        self.update_topic_details()
        self.update_status()

    def action_hide(self) -> None:
        """Hide the selected topic, node, or service from the dashboard."""
        tab = self.active_tab
        if tab in ("topics", "nodes", "services") and self.selected[tab]:
            self.hidden.add(self.selected[tab])
            self.refresh_graph()

    def action_edit_parameter(self) -> None:
        node_name = self.selected.get("nodes") if self.active_tab == "nodes" else None
        if not node_name:
            self.notify_control("Select a node first", error=True)
            return
        parameters = self.node_parameters.get(node_name, {})
        default_name = next(iter(parameters), "")
        default_value = yaml_inline(parameters.get(default_name))
        self.push_screen(
            CommandForm(
                f"Set parameter on {node_name}",
                [("parameter name", default_name)],
                default_value,
            ),
            lambda result: self.set_remote_parameter(node_name, result),
        )

    def action_call_service(self) -> None:
        service_name = self.selected.get("services") if self.active_tab == "services" else None
        service = self.services.get(service_name)
        if not service_name or not service or not service["types"]:
            self.notify_control("Select a service first", error=True)
            return
        type_name = service["types"][0]
        try:
            payload = message_to_yaml(get_service(type_name).Request())
        except Exception:  # noqa: BLE001
            payload = "{}"
        self.push_screen(
            CommandForm(f"Call {service_name}\n{type_name}", [], payload),
            lambda result: self.call_dynamic_service(service_name, type_name, result),
        )

    def action_publish_topic(self) -> None:
        topic_name = self.selected.get("topics") if self.active_tab == "topics" else None
        topic = self.topics.get(topic_name)
        if not topic_name or not topic or not topic["types"]:
            self.notify_control("Select a topic first", error=True)
            return
        type_name = topic["types"][0]
        try:
            payload = message_to_yaml(get_message(type_name)())
        except Exception:  # noqa: BLE001
            payload = "{}"
        self.push_screen(
            CommandForm(f"Publish once to {topic_name}\n{type_name}", [], payload),
            lambda result: self.publish_dynamic_message(topic_name, type_name, result),
        )

    def action_record_bag(self) -> None:
        default_topic = self.selected.get("topics") or "-a"
        output = datetime.now().strftime("rosbag_%Y%m%d_%H%M%S")
        self.push_screen(
            CommandForm(
                "Start rosbag recording",
                [("topics, or -a", default_topic), ("output directory", output)],
            ),
            self.start_bag,
        )

    def action_launch(self) -> None:
        self.push_screen(
            CommandForm(
                "Start ROS 2 launch file",
                [("package", ""), ("launch file", "")],
                "",
            ),
            self.start_launch,
        )

    def action_stop_processes(self) -> None:
        if not self.processes:
            self.notify_control("No managed processes are running", error=True)
            return
        self.push_screen(
            CommandForm("Stop all processes started by this app?", [], ""),
            self.stop_managed_processes,
        )

    def action_focus_filter(self) -> None:
        if self.active_tab == "settings":
            return
        self.query_one(f"#filter_{self.active_tab}", Input).focus()

    def action_command_palette(self) -> None:
        candidates = []
        for name, topic in sorted(self.topics.items()):
            candidates.append(("topic", name, ", ".join(topic["types"])))
        for name, node in sorted(self.nodes.items()):
            detail = f"pub:{len(node['pubs'])} sub:{len(node['subs'])} srv:{len(node['services'])}"
            candidates.append(("node", name, detail))
        for name, srv in sorted(self.services.items()):
            candidates.append(("service", name, ", ".join(srv["types"])))
        for name, action in sorted(self.actions.items()):
            candidates.append(("action", name, ", ".join(action["types"])))
        self.push_screen(CommandPalette(candidates), self.on_command_palette)

    def on_command_palette(self, result: str | None) -> None:
        if not result:
            return
        kind, name = result.split(":", 1)
        tab_map = {"topic": "topics", "node": "nodes", "service": "services", "action": "actions"}
        tab = tab_map.get(kind)
        if not tab:
            return
        self.query_one(f"#filter_{tab}", Input).value = name
        self.selected[tab] = name
        self.fill_table(tab)
        if tab == "topics":
            self.start_echo(name)
        elif tab == "nodes":
            self.request_node_parameters(name)
            self.request_node_lifecycle(name)
        self.update_details(tab)
        self.query_one(TabbedContent).active = tab
        self.action_switch_tab(tab)

    def action_switch_tab(self, tab: str) -> None:
        self.query_one(TabbedContent).active = tab
        if tab == "topics":
            self.query_one("#table_topics", DataTable).focus()
        elif tab == "graph":
            self.query_one("#graph_canvas", GraphCanvas).focus()
        elif tab in ("tf", "logs", "diagnostics", "operations"):
            self.query_one(f"#{tab}_scroll", VerticalScroll).focus()
        elif tab == "settings":
            self.query_one("#settings_scroll", VerticalScroll).focus()
        else:
            self.query_one(f"#table_{tab}", DataTable).focus()

    def apply_settings(self) -> None:
        theme = self.query_one("#theme_select", Select).value
        self.graph_style = self.query_one("#graph_style_select", Select).value
        refresh_text = self.query_one("#refresh_input", Input).value
        try:
            new_refresh = float(refresh_text)
            if new_refresh <= 0:
                raise ValueError
        except ValueError:
            self.notify_control("Refresh interval must be a positive number", error=True)
            return
        self.refresh_seconds = new_refresh
        self.refresh_timer.stop()
        self.refresh_timer = self.set_interval(self.refresh_seconds, self.refresh_graph)
        if self.paused:
            self.refresh_timer.pause()
        self.app.theme = f"textual-{theme}"
        self.app.dark = theme == "dark"
        if self.active_tab == "graph":
            self.update_graph()
        self.notify_control(
            f"Settings: theme={theme}, graph={self.graph_style}, refresh={self.refresh_seconds:g}s"
        )

    def on_button_pressed(self, event: Button.Pressed) -> None:
        if event.button.id == "apply_settings":
            self.apply_settings()

    @property
    def active_tab(self) -> str:
        return self.query_one(TabbedContent).active or "topics"

    def notify_control(self, message: str, error: bool = False) -> None:
        color = "#ff4f55" if error else "#ffffff"
        self.operation_events.appendleft(message)
        self.query_one("#status_bar", Static).update(f"[{color}]{escape(message)}[/]")
        self.update_operations()

    def set_remote_parameter(self, node_name: str, result: dict | None) -> None:
        if result is None:
            return
        parameter_name = result["fields"][0].strip()
        if not parameter_name:
            self.notify_control("Parameter name is required", error=True)
            return
        try:
            value = yaml.safe_load(result["payload"])
            parameter = Parameter(parameter_name, value=value).to_parameter_msg()
            client = self.node.create_client(SetParameters, f"{node_name}/set_parameters")
            if not client.wait_for_service(timeout_sec=0.1):
                raise RuntimeError("parameter service is unavailable")
            request = SetParameters.Request(parameters=[parameter])
            self.control_clients.append(client)
            future = client.call_async(request)
            future.add_done_callback(
                lambda done: self.on_parameter_set(node_name, parameter_name, done)
            )
        except Exception as exc:  # noqa: BLE001
            self.notify_control(f"Parameter error: {exc}", error=True)

    def on_parameter_set(self, node_name: str, parameter_name: str, future) -> None:
        try:
            response = future.result()
            outcome = response.results[0]
            if not outcome.successful:
                raise RuntimeError(outcome.reason or "parameter rejected")
            self.notify_control(f"Set {node_name}.{parameter_name}")
            self.node_parameters.pop(node_name, None)
            self.parameter_requests.discard(node_name)
            self.request_node_parameters(node_name)
        except Exception as exc:  # noqa: BLE001
            self.notify_control(f"Parameter error: {exc}", error=True)

    def call_dynamic_service(self, service_name: str, type_name: str, result: dict | None) -> None:
        if result is None:
            return
        try:
            values = yaml.safe_load(result["payload"]) or {}
            if not isinstance(values, dict):
                raise TypeError("request YAML must be a mapping")
            service_class = get_service(type_name)
            request = service_class.Request()
            setters = set_message_fields(
                request, values, expand_header_auto=True, expand_time_now=True
            )
            for setter in setters:
                setter(self.node.get_clock().now().to_msg())
            client = self.node.create_client(service_class, service_name)
            if not client.wait_for_service(timeout_sec=0.1):
                raise RuntimeError("service is unavailable")
            self.control_clients.append(client)
            future = client.call_async(request)
            future.add_done_callback(
                lambda done: self.on_service_response(service_name, done)
            )
            self.notify_control(f"Calling {service_name}")
        except Exception as exc:  # noqa: BLE001
            self.notify_control(f"Service error: {exc}", error=True)

    def on_service_response(self, service_name: str, future) -> None:
        try:
            response = future.result()
            summary = message_to_yaml(response).strip().replace("\n", " ")
            self.notify_control(f"{service_name}: {summary[:120]}")
        except Exception as exc:  # noqa: BLE001
            self.notify_control(f"Service error: {exc}", error=True)

    def publish_dynamic_message(
        self, topic_name: str, type_name: str, result: dict | None
    ) -> None:
        if result is None:
            return
        try:
            values = yaml.safe_load(result["payload"]) or {}
            if not isinstance(values, dict):
                raise TypeError("message YAML must be a mapping")
            message_class = get_message(type_name)
            message = message_class()
            setters = set_message_fields(
                message, values, expand_header_auto=True, expand_time_now=True
            )
            for setter in setters:
                setter(self.node.get_clock().now().to_msg())
            publisher = self.node.create_publisher(message_class, topic_name, 10)
            self.ephemeral_publishers.append(publisher)
            publisher.publish(message)
            self.set_timer(0.15, lambda: publisher.publish(message))
            self.set_timer(0.5, lambda: self.release_publisher(publisher))
            self.notify_control(f"Published once to {topic_name}")
        except Exception as exc:  # noqa: BLE001
            self.notify_control(f"Publish error: {exc}", error=True)

    def release_publisher(self, publisher) -> None:
        if publisher in self.ephemeral_publishers:
            self.ephemeral_publishers.remove(publisher)
            self.node.destroy_publisher(publisher)

    def start_bag(self, result: dict | None) -> None:
        if result is None:
            return
        topics, output = (field.strip() for field in result["fields"])
        try:
            executable = shutil.which("ros2")
            if not executable or not output:
                raise RuntimeError("ros2 or output directory is unavailable")
            topic_args = shlex.split(topics) or ["-a"]
            command = [executable, "bag", "record", "-o", output, *topic_args]
            process = subprocess.Popen(
                command,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
            self.processes[f"bag:{output}"] = process
            self.notify_control(f"Recording bag to {output}")
        except Exception as exc:  # noqa: BLE001
            self.notify_control(f"Rosbag error: {exc}", error=True)

    def start_launch(self, result: dict | None) -> None:
        if result is None:
            return
        package, launch_file = (field.strip() for field in result["fields"])
        try:
            executable = shutil.which("ros2")
            if not executable or not package or not launch_file:
                raise RuntimeError("package and launch file are required")
            command = [
                executable, "launch", package, launch_file,
                *shlex.split(result["payload"]),
            ]
            process = subprocess.Popen(
                command,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
            self.processes[f"launch:{package}/{launch_file}"] = process
            self.notify_control(f"Started {package}/{launch_file}")
        except Exception as exc:  # noqa: BLE001
            self.notify_control(f"Launch error: {exc}", error=True)

    def stop_managed_processes(self, result: dict | None) -> None:
        if result is None:
            return
        stopped = 0
        for label, process in list(self.processes.items()):
            if process.poll() is None:
                try:
                    os.killpg(os.getpgid(process.pid), signal.SIGINT)
                    stopped += 1
                except ProcessLookupError:
                    pass
            self.processes.pop(label, None)
        self.notify_control(f"Stopped {stopped} managed process(es)")

    # ------------------------------------------------------------ ROS graph

    def spin_once(self) -> None:
        if rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0)

    def refresh_graph(self) -> None:
        """Snapshot topics, nodes and services from the ROS graph."""
        try:
            self.collect_graph()
        except Exception as exc:  # noqa: BLE001 - surface any rclpy failure
            self.query_one("#status_bar", Static).update(f"Graph error: {escape(str(exc))}")
            return

        domain = os.environ.get("ROS_DOMAIN_ID", "0")
        visible_nodes = [n for n in self.nodes if n not in self.hidden]
        visible_topics = [t for t in self.topics if t not in self.hidden]
        visible_services = [s for s in self.services if s not in self.hidden]
        self.query_one("#graph_status", Static).update(
            f"● domain {domain}  ·  {len(visible_nodes)} nodes  ·  "
            f"{len(visible_topics)} topics  ·  {len(visible_services)} services"
        )

        for tab in TAB_IDS:
            self.fill_table(tab)
        self.update_details(self.active_tab)
        self.update_status()

    def collect_graph(self) -> None:
        """Snapshot the graph, hiding this app's own node and endpoints."""
        node = self.node
        me = node.get_fully_qualified_name()

        def not_me(endpoints):
            return [e for e in endpoints if full_name(e.node_name, e.node_namespace) != me]

        topics = {}
        for name, types in node.get_topic_names_and_types():
            topics[name] = {
                "types": types,
                "pubs": not_me(node.get_publishers_info_by_topic(name)),
                "subs": not_me(node.get_subscriptions_info_by_topic(name)),
            }

        nodes = {}
        node_names = node.get_node_names_and_namespaces()
        for name, ns in node_names:
            fq = full_name(name, ns)
            if fq == me:
                continue
            try:
                nodes[fq] = {
                    "pubs": node.get_publisher_names_and_types_by_node(name, ns),
                    "subs": node.get_subscriber_names_and_types_by_node(name, ns),
                    "services": node.get_service_names_and_types_by_node(name, ns),
                    "clients": node.get_client_names_and_types_by_node(name, ns),
                }
            except Exception:  # node may have vanished mid-query
                continue

        services = {}
        for name, types in node.get_service_names_and_types():
            services[name] = {"types": types, "servers": []}
        for fq, info in nodes.items():
            for srv, _ in info["services"]:
                if srv in services:
                    services[srv]["servers"].append(fq)

        actions = {}
        if _ACTIONS_AVAILABLE:
            try:
                actions = {
                    action_name: {"types": types, "servers": [], "clients": []}
                    for action_name, types in get_action_names_and_types(node)
                }
            except Exception:
                actions = {}
            for name, ns in node_names:
                fq = full_name(name, ns)
                if fq == me:
                    continue
                try:
                    servers = get_action_server_names_and_types_by_node(
                        node, name, ns
                    )
                    for action_name, _ in servers:
                        actions.setdefault(
                            action_name, {"types": [], "servers": [], "clients": []}
                        )
                        actions[action_name]["servers"].append(fq)
                    clients = get_action_client_names_and_types_by_node(
                        node, name, ns
                    )
                    for action_name, _ in clients:
                        actions.setdefault(
                            action_name, {"types": [], "servers": [], "clients": []}
                        )
                        actions[action_name]["clients"].append(fq)
                except Exception:
                    continue

        self.topics, self.nodes, self.services, self.actions = topics, nodes, services, actions

    # ----------------------------------------------------------------- tables

    def rows_for(self, tab: str) -> list[tuple[str, tuple]]:
        if tab == "topics":
            return [
                (
                    name,
                    (
                        topic_name_cell(name),
                        topic_type_cell(t["types"]),
                        topic_activity_cell(len(t["pubs"]), len(t["subs"])),
                    ),
                )
                for name, t in sorted(self.topics.items())
            ]
        if tab == "nodes":
            return [
                (
                    name,
                    (
                        name,
                        len(n["pubs"]),
                        len(n["subs"]),
                        len(n["services"]),
                        len(n["clients"]),
                    ),
                )
                for name, n in sorted(self.nodes.items())
            ]
        if tab == "services":
            return [
                (name, (name, ", ".join(s["types"]), len(s["servers"])))
                for name, s in sorted(self.services.items())
            ]
        if tab == "actions":
            return [
                (
                    name,
                    (
                        name,
                        ", ".join(a["types"]),
                        len(a["servers"]),
                        len(a["clients"]),
                    ),
                )
                for name, a in sorted(self.actions.items())
            ]
        return []

    def fill_table(self, tab: str) -> None:
        """Rebuild a graph view from cache, keeping its selection."""
        needle = self.query_one(f"#filter_{tab}", Input).value.lower()
        if tab == "graph":
            self.update_graph(needle)
            return
        if tab == "tf":
            self.update_tf(needle)
            return
        if tab == "operations":
            self.update_operations()
            return
        if tab == "logs":
            self.update_logs(needle)
            return
        if tab == "diagnostics":
            self.update_diagnostics(needle)
            return

        table = self.query_one(f"#table_{tab}", DataTable)
        rows = [
            (key, row)
            for key, row in self.rows_for(tab)
            if key not in self.hidden and needle in self._searchable(tab, key).lower()
        ]

        table.clear()
        for key, row in rows:
            cells = [escape(str(cell)) if isinstance(cell, str) else cell for cell in row]
            table.add_row(*cells, height=4 if tab == "topics" else 1, key=key)

        keys = [key for key, _ in rows]
        current = self.selected[tab]
        if current in keys:
            table.move_cursor(row=keys.index(current))
        else:
            self.select(tab, keys[0] if keys else None)

    def _searchable(self, tab: str, key: str) -> str:
        if tab == "topics":
            return " ".join([key, *self.topics[key]["types"]])
        if tab == "actions":
            action = self.actions[key]
            return " ".join([key, *action["types"], *action["servers"], *action["clients"]])
        return key

    def update_graph(self, needle: str = "") -> None:
        matches = []
        for name, topic in sorted(self.topics.items()):
            if name in self.hidden:
                continue
            publishers = sorted(
                full_name(ep.node_name, ep.node_namespace)
                for ep in topic["pubs"]
                if full_name(ep.node_name, ep.node_namespace) not in self.hidden
            )
            subscribers = sorted(
                full_name(ep.node_name, ep.node_namespace)
                for ep in topic["subs"]
                if full_name(ep.node_name, ep.node_namespace) not in self.hidden
            )
            searchable = " ".join([name, *topic["types"], *publishers, *subscribers]).lower()
            if needle not in searchable:
                continue
            matches.append((name, topic, publishers, subscribers))

        if not matches:
            self.query_one("#graph_view", Static).update(
                Text("No matching graph connections.", style="#666666")
            )
            return

        if self.graph_style == "tree":
            tree = Tree("graph", style="bold #f5f5f7")
            for name, topic, publishers, subscribers in matches:
                topic_branch = tree.add(
                    Text(name, style="bold #00b7ff"), style="bold #00b7ff"
                )

                pub_node = topic_branch.add(
                    Text("publishes", style="dim #636366"), style="dim #636366"
                )
                if publishers:
                    for pub in publishers:
                        pub_node.add(Text(pub, style="#f5f5f7"))
                else:
                    pub_node.add(Text("missing publisher", style="bold #ff4f55"))

                sub_node = topic_branch.add(
                    Text("subscribes", style="dim #636366"), style="dim #636366"
                )
                if subscribers:
                    for sub in subscribers:
                        sub_node.add(Text(sub, style="#f5f5f7"))
                else:
                    sub_node.add(Text("missing subscriber", style="bold #ffb000"))
            self.query_one("#graph_view", Static).update(tree)
        else:
            flows = []
            for name, topic, publishers, subscribers in matches:
                publisher_text = Text("\n".join(publishers), style="#d7d7d7")
                if not publishers:
                    publisher_text = Text("missing publisher", style="bold #ff4f55")

                topic_text = Text(name, style="bold #00b7ff")
                topic_text.append("\n")
                topic_text.append(", ".join(topic["types"]), style="#666666")

                subscriber_text = Text("\n".join(subscribers), style="#d7d7d7")
                if not subscribers:
                    subscriber_text = Text("missing subscriber", style="bold #ffb000")

                flow = Table.grid(expand=True, padding=(0, 1))
                flow.add_column(ratio=2)
                flow.add_column(width=6, justify="center")
                flow.add_column(ratio=3)
                flow.add_column(width=6, justify="center")
                flow.add_column(ratio=2)
                flow.add_row(
                    publisher_text,
                    Text("────▶", style="#ffffff" if publishers else "#ff4f55"),
                    topic_text,
                    Text("────▶", style="#ffffff" if subscribers else "#ffb000"),
                    subscriber_text,
                )
                flows.extend((flow, Text("")))

            graph = Group(*flows[:-1])
            self.query_one("#graph_view", Static).update(graph)

    def on_tf(self, msg, is_static: bool) -> None:
        now = time.monotonic()
        for transform in msg.transforms:
            parent = transform.header.frame_id.lstrip("/") or "world"
            child = transform.child_frame_id.lstrip("/")
            if not child:
                continue
            edge = self.tf_edges.setdefault(
                (parent, child),
                {"times": deque(maxlen=HZ_WINDOW), "static": is_static, "last": now},
            )
            edge["static"] = edge["static"] or is_static
            edge["last"] = now
            if not is_static:
                edge["times"].append(now)

    def update_tf(self, needle: str = "") -> None:
        needle = needle or self.query_one("#filter_tf", Input).value.lower()
        view = self.query_one("#tf_view", Static)
        if not _TF_AVAILABLE:
            view.update(Text("tf2_msgs is not available.", style="#ff4f55"))
            return

        edges = {
            key: value
            for key, value in self.tf_edges.items()
            if not needle or needle in " ".join(key).lower()
        }
        if not edges:
            view.update(Text("No matching transforms.", style="#666666"))
            return

        children: dict[str, list[str]] = {}
        child_frames = set()
        for parent, child in edges:
            children.setdefault(parent, []).append(child)
            child_frames.add(child)
        roots = sorted(set(children) - child_frames) or sorted(children)[:1]
        now = time.monotonic()
        trees = []

        def add_children(branch: Tree, parent: str, seen: set[str]) -> None:
            for child in sorted(children.get(parent, [])):
                edge = edges[(parent, child)]
                age = now - edge["last"]
                if edge["static"]:
                    metric, color = "static", "#8e8e93"
                elif age > 2.0:
                    metric, color = f"stale {age:.1f}s", "#ff4f55"
                elif len(edge["times"]) > 1:
                    span = edge["times"][-1] - edge["times"][0]
                    hz = (len(edge["times"]) - 1) / span if span > 0 else 0.0
                    metric, color = f"{hz:.1f} Hz", "#ffffff"
                else:
                    metric, color = "waiting", "#ffb000"
                label = Text(child, style="#d7d7d7")
                label.append(f"  {metric}", style=color)
                node_branch = branch.add(label)
                if child not in seen:
                    add_children(node_branch, child, seen | {child})

        for root in roots:
            tree = Tree(Text(root, style="bold #00b7ff"), guide_style="#3a3a3c")
            add_children(tree, root, {root})
            trees.extend((tree, Text("")))
        view.update(Group(*trees[:-1]))

    def update_action_details(self) -> None:
        name = self.selected["actions"]
        info = self.query_one("#info_actions", Static)
        action = self.actions.get(name)
        if not action:
            info.update("[dim]No action selected.[/]")
            return
        if not _ACTIONS_AVAILABLE:
            info.update(Text("ROS action graph support is unavailable.", style="#ff4f55"))
            return

        lines = [
            f"[b]{escape(name)}[/]",
            f"[dim]Type[/]  {escape(', '.join(action['types']))}",
        ]
        if action["types"]:
            lines += ["", "[b]Definition[/]"]
            lines += interface_definition_lines(action["types"][0], "action")
        lines += [
            "",
            f"[b]Servers[/] [dim]({len(action['servers'])})[/]",
        ]
        lines += [f"  [#00b7ff]•[/] {escape(s)}" for s in action["servers"]] or ["  [dim]none[/]"]
        lines += [
            "",
            f"[b]Clients[/] [dim]({len(action['clients'])})[/]",
        ]
        lines += [f"  [#00b7ff]•[/] {escape(c)}" for c in action["clients"]] or ["  [dim]none[/]"]
        lines += ["", "[dim]press g to send a goal[/]"]
        info.update("\n".join(lines))

    def update_operations(self) -> None:
        lines = [
            "[b]Safe controls[/]",
            "",
            "[dim]m[/] edit selected node parameter",
            "[dim]c[/] call selected service",
            "[dim]u[/] publish to selected topic",
            "[dim]g[/] send goal to selected action",
            "[dim]L[/] lifecycle transition for selected node",
            "[dim]b[/] start rosbag recording",
            "[dim]l[/] start a launch file",
            "[dim]s[/] stop processes started here",
            "[dim]ctrl+p[/] open global command palette",
            "",
            "[b]Managed processes[/]",
        ]
        active = []
        for label, process in list(self.processes.items()):
            if process.poll() is None:
                active.append(f"  [#ffffff]●[/] {escape(label)}  pid {process.pid}")
            else:
                self.processes.pop(label, None)
        lines += active or ["  [dim]none[/]"]
        lines += ["", "[b]Recent operations[/]"]
        lines += [f"  {escape(event)}" for event in self.operation_events] or ["  [dim]none[/]"]
        lines += ["", "[yellow]State-changing operations require confirmation.[/]"]
        self.query_one("#operations_view", Static).update("\n".join(lines))

    def on_rosout(self, msg) -> None:
        """Cache a /rosout log message."""
        level_name, color = LOG_LEVELS.get(msg.level, ("UNKNOWN", "#8e8e93"))
        stamp = f"{msg.stamp.sec:>10}.{msg.stamp.nanosec:09d}"[:15]
        line = Text(stamp, style="#666666")
        line.append(" ")
        line.append(f"[{level_name:>5}]", style=f"bold {color}")
        line.append(" ")
        line.append(f"[{escape(msg.name)}]", style="#888888")
        line.append(" ")
        line.append(escape(msg.msg), style="#d7d7d7")
        self.log_lines.append(line)

    def on_diagnostics(self, msg) -> None:
        """Cache the latest diagnostic status for each key."""
        for s in msg.status:
            key = f"{s.hardware_id}/{s.name}" if s.hardware_id else s.name
            level = s.level[0] if isinstance(s.level, bytes) else s.level
            self.diagnostics[key] = {
                "level": level,
                "name": s.name,
                "hardware_id": s.hardware_id,
                "message": s.message,
                "values": {v.key: v.value for v in s.values},
            }

    def update_logs(self, needle: str = "") -> None:
        """Render the filtered /rosout log stream."""
        needle = needle or self.query_one("#filter_logs", Input).value.lower()
        filtered = [
            line for line in self.log_lines if needle in line.plain.lower()
        ]
        view = self.query_one("#log_view", Static)
        if not filtered:
            view.update(Text("No matching log messages.", style="#666666"))
            return
        view.update(Group(*reversed(filtered[:LOG_MAX_LINES])))

    def update_diagnostics(self, needle: str = "") -> None:
        """Render the filtered diagnostics summary."""
        needle = needle or self.query_one("#filter_diagnostics", Input).value.lower()
        view = self.query_one("#diagnostics_view", Static)
        if not _DIAGNOSTICS_AVAILABLE:
            view.update(Text("diagnostic_msgs is not available.", style="#ff4f55"))
            return

        def severity(key: str) -> int:
            return self.diagnostics[key]["level"]

        ordered = sorted(
            self.diagnostics,
            key=lambda k: (-severity(k), self.diagnostics[k]["name"]),
        )
        filtered = [
            k for k in ordered
            if needle in " ".join([
                self.diagnostics[k]["name"],
                self.diagnostics[k]["hardware_id"],
                self.diagnostics[k]["message"],
            ]).lower()
        ]

        if not filtered:
            view.update(Text("No matching diagnostics.", style="#666666"))
            return

        lines: list[Text] = []
        for key in filtered:
            d = self.diagnostics[key]
            level_name, color = DIAGNOSTIC_LEVELS.get(d["level"], ("?", "#8e8e93"))
            row = Text(level_name, style=f"bold {color}")
            row.append("  ")
            row.append(escape(d["name"]), style="#d7d7d7")
            if d["hardware_id"]:
                row.append("  ")
                row.append(f"[{escape(d['hardware_id'])}]", style="#666666")
            if d["message"]:
                row.append("  ")
                row.append(escape(d["message"]), style="#aaaaaa")
            if d["values"]:
                pairs = [f"{k}={v}" for k, v in d["values"].items()]
                row.append("  ")
                row.append(" · ".join(pairs), style="#777777")
            lines.append(row)
        view.update(Group(*lines))

    def on_input_changed(self, event: Input.Changed) -> None:
        if not event.input.id or not event.input.id.startswith("filter_"):
            return
        tab = event.input.id.removeprefix("filter_")
        self.fill_table(tab)

    def on_input_submitted(self, event: Input.Submitted) -> None:
        if not event.input.id or not event.input.id.startswith("filter_"):
            return
        tab = event.input.id.removeprefix("filter_")
        if tab == "topics":
            self.query_one("#table_topics", DataTable).focus()
        elif tab == "graph":
            self.query_one("#graph_canvas", GraphCanvas).focus()
        elif tab in ("tf", "logs", "diagnostics", "operations"):
            self.query_one(f"#{tab}_scroll", VerticalScroll).focus()
        else:
            self.query_one(f"#table_{tab}", DataTable).focus()

    def on_data_table_row_highlighted(self, event: DataTable.RowHighlighted) -> None:
        tab = event.data_table.id.removeprefix("table_")
        key = event.row_key.value if event.row_key is not None else None
        self.select(tab, key)

    def on_tabbed_content_tab_activated(self, event: TabbedContent.TabActivated) -> None:
        self.update_details(self.active_tab)

    def select(self, tab: str, key) -> None:
        if self.selected[tab] == key:
            return
        self.selected[tab] = key
        if tab == "topics":
            self.start_echo(key)
        elif tab == "nodes" and key:
            self.request_node_parameters(key)
            self.request_node_lifecycle(key)
        self.update_details(tab)

    # ---------------------------------------------------------------- details

    def update_details(self, tab: str) -> None:
        if tab == "topics":
            self.update_topic_details()
        elif tab == "nodes":
            self.update_node_details()
        elif tab == "services":
            self.update_service_details()
        elif tab == "actions":
            self.update_action_details()

    def update_topic_details(self) -> None:
        name = self.selected["topics"]
        info = self.query_one("#info_topics", Static)
        topic = self.topics.get(name)
        if not topic:
            info.update("[dim]No topic selected.[/]")
            return

        lines = [
            f"[b]{escape(name)}[/]",
            f"[dim]Type[/]      {escape(', '.join(topic['types']))}",
        ]
        if topic["types"]:
            lines += ["", "[b]Definition[/]"]
            lines += interface_definition_lines(topic["types"][0], "msg")
        lines += [
            "",
            f"[dim]Rate[/]      {self.hz_text()}",
            f"[dim]Bandwidth[/] {self.bandwidth_text()}",
            "",
            f"[b]Publishers[/] [dim]({len(topic['pubs'])})[/]",
        ]
        lines += self.endpoint_lines(topic["pubs"])
        lines += ["", f"[b]Subscribers[/] [dim]({len(topic['subs'])})[/]"]
        lines += self.endpoint_lines(topic["subs"])

        echo_state = "on" if self.echo_enabled else "off"
        lines += [
            "",
            f"[b]Echo[/] [dim]({echo_state}, {self.msg_count} msgs)[/]",
        ]
        if not self.echo_enabled:
            lines.append("  [dim]press e to enable[/]")
        elif self.echo_error:
            lines.append(f"  [red]{escape(self.echo_error)}[/]")
        elif self.last_msg is None:
            lines.append("  [dim]waiting for messages...[/]")
        else:
            try:
                yaml_lines = message_to_yaml(self.last_msg).rstrip().splitlines()
            except Exception as exc:  # noqa: BLE001
                yaml_lines = [f"<unprintable: {exc}>"]
            if len(yaml_lines) > ECHO_MAX_LINES:
                remaining = len(yaml_lines) - ECHO_MAX_LINES
                yaml_lines = yaml_lines[:ECHO_MAX_LINES] + [
                    f"... ({remaining} more lines)"
                ]
            lines += [f"  {escape(line)}" for line in yaml_lines]

        info.update("\n".join(lines))

    def endpoint_lines(self, endpoints) -> list[str]:
        if not endpoints:
            return ["  [dim]none[/]"]
        out = []
        for ep in endpoints:
            node = full_name(ep.node_name, ep.node_namespace)
            out.append(f"  [#00b7ff]•[/] {escape(node)}")
            out.append(f"    [dim]{escape(qos_summary(ep.qos_profile))}[/]")
        return out

    def request_node_parameters(self, node_name: str) -> None:
        if node_name in self.parameter_requests or node_name in self.node_parameters:
            return
        try:
            client = self.node.create_client(ListParameters, f"{node_name}/list_parameters")
            if not client.wait_for_service(timeout_sec=0):
                return
            self.parameter_requests.add(node_name)
            self.control_clients.append(client)
            request = ListParameters.Request(prefixes=[], depth=0)
            future = client.call_async(request)
            future.add_done_callback(lambda done: self.on_parameter_list(node_name, done))
        except Exception:
            self.parameter_requests.discard(node_name)

    def on_parameter_list(self, node_name: str, future) -> None:
        try:
            names = list(future.result().result.names)
            if not names:
                self.node_parameters[node_name] = {}
                self.parameter_requests.discard(node_name)
                return
            client = self.node.create_client(GetParameters, f"{node_name}/get_parameters")
            self.control_clients.append(client)
            request = GetParameters.Request(names=names)
            next_future = client.call_async(request)
            next_future.add_done_callback(
                lambda done: self.on_parameter_values(node_name, names, done)
            )
        except Exception:
            self.parameter_requests.discard(node_name)

    def on_parameter_values(self, node_name: str, names: list[str], future) -> None:
        try:
            values = future.result().values
            self.node_parameters[node_name] = {
                name: parameter_value_to_python(value)
                for name, value in zip(names, values)
            }
            if self.selected["nodes"] == node_name:
                self.update_node_details()
        finally:
            self.parameter_requests.discard(node_name)

    def request_node_lifecycle(self, node_name: str) -> None:
        if not _LIFECYCLE_AVAILABLE:
            return
        if node_name in self.lifecycle_requests or node_name in self.node_lifecycle:
            return
        try:
            client = self.node.create_client(GetState, f"{node_name}/get_state")
            if not client.wait_for_service(timeout_sec=0):
                return
            self.lifecycle_requests.add(node_name)
            self.control_clients.append(client)
            future = client.call_async(GetState.Request())
            future.add_done_callback(lambda done: self.on_lifecycle_state(node_name, done))
        except Exception:
            self.lifecycle_requests.discard(node_name)

    def on_lifecycle_state(self, node_name: str, future) -> None:
        try:
            response = future.result()
            state = response.current_state
            self.node_lifecycle[node_name] = {
                "id": state.id,
                "label": state.label,
            }
        except Exception:
            self.node_lifecycle[node_name] = None
        finally:
            self.lifecycle_requests.discard(node_name)
        if self.selected["nodes"] == node_name:
            self.update_node_details()

    def update_node_details(self) -> None:
        name = self.selected["nodes"]
        info = self.query_one("#info_nodes", Static)
        node = self.nodes.get(name)
        if not node:
            info.update("[dim]No node selected.[/]")
            return

        lines = [f"[b]{escape(name)}[/]"]
        for title, key in (
            ("Publishers", "pubs"),
            ("Subscriptions", "subs"),
            ("Services", "services"),
            ("Clients", "clients"),
        ):
            entries = sorted(node[key])
            lines += ["", f"[b]{title}[/] [dim]({len(entries)})[/]"]
            if not entries:
                lines.append("  [dim]none[/]")
            for topic, types in entries:
                lines.append(f"  [#00b7ff]•[/] {escape(topic)}")
                lines.append(f"    [dim]{escape(', '.join(types))}[/]")
        lifecycle = self.node_lifecycle.get(name)
        lines += ["", "[b]Lifecycle[/] [dim](L to transition)[/]"]
        if lifecycle is None:
            lines.append("  [dim]unknown[/]")
        elif not lifecycle:
            lines.append("  [dim]not a lifecycle node[/]")
        else:
            lines.append(f"  State: {escape(lifecycle['label'])} [dim](id={lifecycle['id']})[/]")
        parameters = self.node_parameters.get(name)
        lines += ["", "[b]Parameters[/] [dim](m to edit)[/]"]
        if parameters is None:
            lines.append("  [dim]loading or unavailable[/]")
        elif not parameters:
            lines.append("  [dim]none[/]")
        else:
            for parameter_name, value in sorted(parameters.items()):
                rendered = yaml_inline(value)
                lines.append(f"  {escape(parameter_name)} [dim]= {escape(rendered)}[/]")
        info.update("\n".join(lines))

    def action_lifecycle_transition(self) -> None:
        node_name = self.selected.get("nodes")
        if not node_name:
            self.notify_control("Select a node first", error=True)
            return
        lifecycle = self.node_lifecycle.get(node_name)
        if not lifecycle:
            self.notify_control(f"{node_name} does not appear to be a lifecycle node", error=True)
            return
        self.push_screen(
            CommandForm(
                "Lifecycle transition",
                [("transition", "configure")],
                "",
            ),
            lambda result: self.change_lifecycle_state(node_name, result),
        )

    def change_lifecycle_state(self, node_name: str, result: dict | None) -> None:
        if result is None or not _LIFECYCLE_AVAILABLE:
            return
        transition_name = result["fields"][0].strip().lower()
        transition_id = LIFECYCLE_TRANSITIONS.get(transition_name)
        if transition_id is None:
            self.notify_control(f"Unknown transition: {transition_name}", error=True)
            return
        try:
            client = self.node.create_client(ChangeState, f"{node_name}/change_state")
            if not client.wait_for_service(timeout_sec=0.2):
                self.notify_control("change_state service is unavailable", error=True)
                return
            request = ChangeState.Request()
            request.transition.id = transition_id
            request.transition.label = transition_name
            self.control_clients.append(client)
            future = client.call_async(request)
            future.add_done_callback(
                lambda done: self.on_lifecycle_changed(node_name, transition_name, done)
            )
        except Exception as exc:
            self.notify_control(f"Lifecycle error: {exc}", error=True)

    def on_lifecycle_changed(self, node_name: str, transition_name: str, future) -> None:
        try:
            response = future.result()
            if response.success:
                self.notify_control(f"Transition '{transition_name}' sent to {node_name}")
                self.node_lifecycle.pop(node_name, None)
                self.request_node_lifecycle(node_name)
            else:
                self.notify_control(
                    f"Transition '{transition_name}' failed for {node_name}", error=True
                )
        except Exception as exc:
            self.notify_control(f"Lifecycle error: {exc}", error=True)

    def action_send_action_goal(self) -> None:
        if not _ACTION_CLIENT_AVAILABLE:
            self.notify_control("Action client support is unavailable", error=True)
            return
        action_name = self.selected.get("actions")
        action = self.actions.get(action_name) if action_name else None
        if not action or not action["types"]:
            self.notify_control("Select an action first", error=True)
            return
        self.push_screen(
            CommandForm(
                f"Send goal to {action_name}",
                [],
                "",
            ),
            lambda result: self.send_action_goal(action_name, action["types"][0], result),
        )

    def send_action_goal(self, action_name: str, type_name: str, result: dict | None) -> None:
        if result is None:
            return
        goal_yaml = result.get("payload", "")
        try:
            action_cls = get_action(type_name)
            goal_msg = action_cls.Goal()
            if goal_yaml.strip():
                set_message_fields(goal_msg, yaml.safe_load(goal_yaml) or {})
            client = ActionClient(self.node, action_cls, action_name)
            if not client.wait_for_server(timeout_sec=0.5):
                self.notify_control(f"Action server {action_name} not available", error=True)
                return
            self.action_clients.append(client)
            self.notify_control(f"Sending goal to {action_name}...")
            send_goal_future = client.send_goal_async(
                goal_msg,
                feedback_callback=lambda fb: self.on_action_feedback(action_name, fb),
            )
            send_goal_future.add_done_callback(
                lambda done: self.on_action_goal_response(action_name, done)
            )
        except Exception as exc:
            self.notify_control(f"Action goal error: {exc}", error=True)

    def on_action_goal_response(self, action_name: str, future) -> None:
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.notify_control(f"Goal rejected by {action_name}", error=True)
                return
            self.notify_control(f"Goal accepted by {action_name}")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda done: self.on_action_result(action_name, done)
            )
        except Exception as exc:
            self.notify_control(f"Goal response error: {exc}", error=True)

    def on_action_result(self, action_name: str, future) -> None:
        try:
            result = future.result().result
            self.notify_control(f"Goal finished on {action_name}: {result}")
        except Exception as exc:
            self.notify_control(f"Goal result error on {action_name}: {exc}", error=True)

    def on_action_feedback(self, action_name: str, feedback) -> None:
        self.notify_control(f"Feedback from {action_name}")

    def update_service_details(self) -> None:
        name = self.selected["services"]
        info = self.query_one("#info_services", Static)
        srv = self.services.get(name)
        if not srv:
            info.update("[dim]No service selected.[/]")
            return

        lines = [
            f"[b]{escape(name)}[/]",
            f"[dim]Type[/]  {escape(', '.join(srv['types']))}",
        ]
        if srv["types"]:
            lines += ["", "[b]Definition[/]"] + interface_definition_lines(srv["types"][0], "srv")
        lines += [
            "",
            f"[b]Servers[/] [dim]({len(srv['servers'])})[/]",
        ]
        lines += [f"  [#00b7ff]•[/] {escape(s)}" for s in srv["servers"]] or ["  [dim]none[/]"]
        info.update("\n".join(lines))

    def update_status(self) -> None:
        now = datetime.now().strftime("%H:%M:%S")
        state = "[#ffb000]paused[/]" if self.paused else f"refresh {self.refresh_seconds:g}s"
        echo = "on" if self.echo_enabled else "off"
        self.query_one("#status_bar", Static).update(
            f"{now}  ·  {state}  ·  echo {echo}  ·  "
            "[dim]/ filter  1-9/0 views  r refresh  m/c/g/L  b/l/s  ^p[/]"
        )

    # ------------------------------------------------------------ echo / Hz

    def start_echo(self, topic_name) -> None:
        self.stop_echo()
        if not self.echo_enabled or not topic_name:
            return
        topic = self.topics.get(topic_name)
        if not topic or not topic["types"]:
            return

        type_name = topic["types"][0]
        try:
            msg_cls = get_message(type_name)
        except Exception as exc:  # noqa: BLE001 - type not installed locally
            self.echo_error = f"cannot load {type_name}: {exc}"
            return

        self.sparkline_path = []
        self.sparkline_values.clear()
        self._set_sparkline_visibility(False)

        # Best-effort is compatible with both reliable and best-effort publishers.
        # Mirror durability so latched (transient local) topics still deliver.
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        if topic["pubs"] and all(
            p.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for p in topic["pubs"]
        ):
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.echo_topic = topic_name
        self.echo_sub = self.node.create_subscription(
            msg_cls, topic_name, self.on_echo_message, qos
        )

    def stop_echo(self) -> None:
        echo_sub = getattr(self, "echo_sub", None)
        node = getattr(self, "node", None)
        if echo_sub is not None and node is not None:
            node.destroy_subscription(echo_sub)
        self.echo_sub = None
        self.echo_topic = None
        self.echo_error = None
        self.last_msg = None
        self.msg_count = 0
        if hasattr(self, "msg_times"):
            self.msg_times.clear()
        if hasattr(self, "msg_sizes"):
            self.msg_sizes.clear()
        if hasattr(self, "sparkline_values"):
            self.sparkline_values.clear()
            self.sparkline_path = []
            self._set_sparkline_visibility(False)

    def on_echo_message(self, msg) -> None:
        now = time.monotonic()
        self.last_msg = msg
        self.msg_count += 1
        self.msg_times.append(now)
        try:
            self.msg_sizes.append((now, len(serialize_message(msg))))
        except Exception:  # noqa: BLE001
            pass
        self._update_sparkline(msg)

    def _update_sparkline(self, msg) -> None:
        try:
            if not self.sparkline_path:
                result = _first_numeric_field(msg)
                if result is None:
                    return
                value, path = result
                self.sparkline_path = list(path)
                self.sparkline_values.append(float(value))
                self._set_sparkline_visibility(True)
            else:
                value = msg
                for segment in self.sparkline_path:
                    value = value[segment] if isinstance(segment, int) else getattr(value, segment)
                self.sparkline_values.append(float(value))
            self.query_one("#topic_sparkline").data = list(self.sparkline_values)
        except Exception:  # noqa: BLE001
            pass

    def _set_sparkline_visibility(self, visible: bool) -> None:
        try:
            self.query_one("#topic_sparkline").display = visible
        except Exception:  # noqa: BLE001
            pass

    def bandwidth_text(self) -> str:
        if len(self.msg_sizes) < 2:
            return "[dim]--[/]"
        span = self.msg_sizes[-1][0] - self.msg_sizes[0][0]
        if span <= 0:
            return "[dim]--[/]"
        bytes_per_second = sum(size for _, size in list(self.msg_sizes)[1:]) / span
        if bytes_per_second >= 1_000_000:
            return f"{bytes_per_second / 1_000_000:.2f} MB/s"
        if bytes_per_second >= 1_000:
            return f"{bytes_per_second / 1_000:.2f} kB/s"
        return f"{bytes_per_second:.0f} B/s"

    def hz_text(self) -> str:
        if not self.echo_enabled:
            return "[dim]echo off[/]"
        if len(self.msg_times) < 2:
            return "[dim]--[/]"
        if time.monotonic() - self.msg_times[-1] > 3.0:
            return "[#ffb000]stale[/]"
        span = self.msg_times[-1] - self.msg_times[0]
        if span <= 0:
            return "[dim]--[/]"
        return f"{(len(self.msg_times) - 1) / span:.2f} Hz"


def main() -> None:
    CommandCenterApp().run()


if __name__ == "__main__":
    main()
