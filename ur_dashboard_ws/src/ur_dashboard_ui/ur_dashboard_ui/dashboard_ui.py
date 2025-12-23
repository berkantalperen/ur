import sys
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from PyQt5 import QtCore, QtWidgets
import rclpy
from rclpy.utilities import remove_ros_args
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_service
from std_srvs.srv import SetBool, Trigger


@dataclass(frozen=True)
class ServiceInfo:
    name: str
    srv_type: str


@dataclass(frozen=True)
class FieldSpec:
    name: str
    label: str
    field_type: str
    widget_type: str
    default: object


@dataclass(frozen=True)
class ServiceActionSpec:
    srv_class: object
    fields: Tuple[FieldSpec, ...]


SUPPORTED_SERVICE_TYPES: Dict[str, ServiceActionSpec] = {
    "std_srvs/srv/Trigger": ServiceActionSpec(
        srv_class=Trigger,
        fields=(),
    ),
    "std_srvs/srv/SetBool": ServiceActionSpec(
        srv_class=SetBool,
        fields=(
            FieldSpec(
                name="data",
                label="Data",
                field_type="boolean",
                widget_type="checkbox",
                default=True,
            ),
        ),
    ),
}

SERVICE_TYPE_CACHE: Dict[str, Optional[ServiceActionSpec]] = {}

BOOLEAN_TYPES = {"boolean"}
STRING_TYPES = {"string", "wstring"}
INTEGER_TYPES = {
    "int8",
    "uint8",
    "int16",
    "uint16",
    "int32",
    "uint32",
    "int64",
    "uint64",
}
FLOAT_TYPES = {"float", "double"}


def _normalize_field_type(field_type: str) -> Optional[str]:
    if field_type.startswith("string<=") or field_type.startswith("string<"):
        return "string"
    if field_type.startswith("wstring<=") or field_type.startswith("wstring<"):
        return "wstring"
    if field_type.startswith("sequence<") or field_type.endswith("]"):
        return None
    return field_type


def _widget_type_for_field(field_type: str) -> Optional[str]:
    if field_type in BOOLEAN_TYPES:
        return "checkbox"
    if field_type in STRING_TYPES | INTEGER_TYPES | FLOAT_TYPES:
        return "line_edit"
    return None


def _field_label(field_name: str) -> str:
    return field_name.replace("_", " ").title()


def get_service_action_spec(srv_type: str) -> Optional[ServiceActionSpec]:
    if srv_type in SERVICE_TYPE_CACHE:
        return SERVICE_TYPE_CACHE[srv_type]

    spec = SUPPORTED_SERVICE_TYPES.get(srv_type)
    if spec:
        SERVICE_TYPE_CACHE[srv_type] = spec
        return spec

    try:
        srv_class = get_service(srv_type)
    except (AttributeError, ModuleNotFoundError, ValueError):
        SERVICE_TYPE_CACHE[srv_type] = None
        return None

    request = srv_class.Request()
    fields: List[FieldSpec] = []
    for name, raw_type in request.get_fields_and_field_types().items():
        field_type = _normalize_field_type(raw_type)
        if not field_type:
            SERVICE_TYPE_CACHE[srv_type] = None
            return None
        widget_type = _widget_type_for_field(field_type)
        if not widget_type:
            SERVICE_TYPE_CACHE[srv_type] = None
            return None
        fields.append(
            FieldSpec(
                name=name,
                label=_field_label(name),
                field_type=field_type,
                widget_type=widget_type,
                default=getattr(request, name),
            )
        )

    spec = ServiceActionSpec(srv_class=srv_class, fields=tuple(fields))
    SERVICE_TYPE_CACHE[srv_type] = spec
    return spec


class DashboardNode(Node):
    def __init__(self) -> None:
        super().__init__("ur_dashboard_ui")
        self._client_cache: Dict[str, object] = {}

    def discover_services(self) -> List[ServiceInfo]:
        services = []
        for name, types in self.get_service_names_and_types():
            if "dashboard" not in name:
                continue
            for srv_type in types:
                if srv_type.startswith("rcl_interfaces/srv/"):
                    continue
                services.append(ServiceInfo(name=name, srv_type=srv_type))
        services.sort(key=lambda item: (item.name, item.srv_type))
        return services

    def get_client(self, service_name: str, srv_class: object) -> object:
        key = f"{service_name}:{srv_class.__name__}"
        if key not in self._client_cache:
            self._client_cache[key] = self.create_client(srv_class, service_name)
        return self._client_cache[key]


class DashboardWindow(QtWidgets.QMainWindow):
    def __init__(self, node: DashboardNode) -> None:
        super().__init__()
        self.node = node
        self.setWindowTitle("UR Dashboard UI")
        self.resize(900, 600)

        self.pending_calls: List[Tuple[ServiceInfo, object]] = []
        self.current_service: Optional[ServiceInfo] = None
        self.field_widgets: Dict[str, QtWidgets.QWidget] = {}
        self.last_unsupported_service: Optional[ServiceInfo] = None

        self._build_ui()
        self._connect_signals()

        self.spin_timer = QtCore.QTimer(self)
        self.spin_timer.timeout.connect(self._spin_once)
        self.spin_timer.start(50)

        self.refresh_timer = QtCore.QTimer(self)
        self.refresh_timer.timeout.connect(self.refresh_services)
        self.refresh_timer.start(2000)

        self.refresh_services()

    def _build_ui(self) -> None:
        splitter = QtWidgets.QSplitter()

        left_panel = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left_panel)
        self.refresh_button = QtWidgets.QPushButton("Refresh Services")
        self.service_list = QtWidgets.QListWidget()
        left_layout.addWidget(self.refresh_button)
        left_layout.addWidget(self.service_list)

        right_panel = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right_panel)

        details_group = QtWidgets.QGroupBox("Action Details")
        details_layout = QtWidgets.QFormLayout(details_group)
        self.service_name_label = QtWidgets.QLabel("-")
        self.service_type_label = QtWidgets.QLabel("-")
        details_layout.addRow("Service", self.service_name_label)
        details_layout.addRow("Type", self.service_type_label)

        self.fields_group = QtWidgets.QGroupBox("Request Fields")
        self.fields_layout = QtWidgets.QFormLayout(self.fields_group)

        self.call_button = QtWidgets.QPushButton("Call")
        self.call_button.setEnabled(False)

        output_group = QtWidgets.QGroupBox("Response Output")
        output_layout = QtWidgets.QVBoxLayout(output_group)
        self.output_text = QtWidgets.QTextEdit()
        self.output_text.setReadOnly(True)
        output_layout.addWidget(self.output_text)

        right_layout.addWidget(details_group)
        right_layout.addWidget(self.fields_group)
        right_layout.addWidget(self.call_button)
        right_layout.addWidget(output_group)

        splitter.addWidget(left_panel)
        splitter.addWidget(right_panel)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 2)

        container = QtWidgets.QWidget()
        container_layout = QtWidgets.QVBoxLayout(container)
        container_layout.addWidget(splitter)
        self.setCentralWidget(container)

    def _connect_signals(self) -> None:
        self.refresh_button.clicked.connect(self.refresh_services)
        self.service_list.currentItemChanged.connect(self._on_service_selected)
        self.call_button.clicked.connect(self._call_selected_service)

    def refresh_services(self) -> None:
        previous = self.current_service
        services = self.node.discover_services()

        self.service_list.blockSignals(True)
        self.service_list.clear()
        for service in services:
            item = QtWidgets.QListWidgetItem(f"{service.name} ({service.srv_type})")
            item.setData(QtCore.Qt.UserRole, service)
            self.service_list.addItem(item)
        self.service_list.blockSignals(False)

        if previous:
            self._restore_selection(previous)
        elif self.service_list.count() > 0:
            self.service_list.setCurrentRow(0)

    def _restore_selection(self, previous: ServiceInfo) -> None:
        for index in range(self.service_list.count()):
            item = self.service_list.item(index)
            service = item.data(QtCore.Qt.UserRole)
            if service == previous:
                self.service_list.setCurrentRow(index)
                return
        self.current_service = None
        self._update_details(None)

    def _on_service_selected(self, current: QtWidgets.QListWidgetItem, _previous: QtWidgets.QListWidgetItem) -> None:
        service = current.data(QtCore.Qt.UserRole) if current else None
        self.current_service = service
        self._update_details(service)

    def _update_details(self, service: Optional[ServiceInfo]) -> None:
        self.service_name_label.setText(service.name if service else "-")
        self.service_type_label.setText(service.srv_type if service else "-")

        self._clear_field_widgets()
        if not service:
            self.call_button.setEnabled(False)
            return

        spec = get_service_action_spec(service.srv_type)
        if not spec:
            self.call_button.setEnabled(False)
            if service != self.last_unsupported_service:
                self.output_text.append(
                    f"Service {service.name} uses unsupported type {service.srv_type}."
                )
                self.last_unsupported_service = service
            return

        self.last_unsupported_service = None
        for field in spec.fields:
            widget = self._create_field_widget(field)
            self.field_widgets[field.name] = widget
            self.fields_layout.addRow(field.label, widget)

        self.call_button.setEnabled(True)

    def _clear_field_widgets(self) -> None:
        while self.fields_layout.rowCount():
            self.fields_layout.removeRow(0)
        self.field_widgets.clear()

    def _create_field_widget(self, field: FieldSpec) -> QtWidgets.QWidget:
        if field.widget_type == "checkbox":
            widget = QtWidgets.QCheckBox()
            widget.setChecked(bool(field.default))
            return widget
        widget = QtWidgets.QLineEdit()
        widget.setText("" if field.default is None else str(field.default))
        return widget

    def _parse_line_edit_value(self, field: FieldSpec, text: str) -> object:
        if text == "" and field.default is not None:
            return field.default
        if field.field_type in STRING_TYPES:
            return text
        if field.field_type in INTEGER_TYPES:
            return int(text)
        if field.field_type in FLOAT_TYPES:
            return float(text)
        raise ValueError("Unsupported field type")

    def _call_selected_service(self) -> None:
        service = self.current_service
        if not service:
            return

        spec = get_service_action_spec(service.srv_type)
        if not spec:
            self.output_text.append(
                f"Cannot call {service.name}; unsupported type {service.srv_type}."
            )
            return

        client = self.node.get_client(service.name, spec.srv_class)
        if not client.wait_for_service(timeout_sec=0.5):
            self.output_text.append(f"Service {service.name} is not available.")
            return

        request = spec.srv_class.Request()
        for field in spec.fields:
            widget = self.field_widgets.get(field.name)
            if isinstance(widget, QtWidgets.QCheckBox):
                setattr(request, field.name, widget.isChecked())
            elif isinstance(widget, QtWidgets.QLineEdit):
                try:
                    value = self._parse_line_edit_value(field, widget.text())
                except ValueError:
                    self.output_text.append(
                        f"Invalid value for {field.label}; expected {field.field_type}."
                    )
                    return
                setattr(request, field.name, value)

        future = client.call_async(request)
        self.pending_calls.append((service, future))
        self.output_text.append(f"Calling {service.name} ({service.srv_type})...")

    def _spin_once(self) -> None:
        rclpy.spin_once(self.node, timeout_sec=0.0)

        completed: List[Tuple[ServiceInfo, object]] = []
        for service, future in self.pending_calls:
            if future.done():
                try:
                    response = future.result()
                except Exception as exc:  # pragma: no cover - runtime ROS errors
                    self.output_text.append(f"{service.name} failed: {exc}")
                else:
                    self.output_text.append(f"{service.name} response: {response}")
                completed.append((service, future))

        for entry in completed:
            self.pending_calls.remove(entry)

    def closeEvent(self, event: QtCore.QEvent) -> None:
        self.spin_timer.stop()
        self.refresh_timer.stop()
        super().closeEvent(event)


def main() -> None:
    rclpy.init(args=sys.argv)
    qt_args = remove_ros_args(sys.argv)
    app = QtWidgets.QApplication(qt_args)
    node = DashboardNode()
    window = DashboardWindow(node)
    window.show()
    exit_code = app.exec_()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
