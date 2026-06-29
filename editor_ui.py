import pygame
import pygame_gui
import json
import socket
import struct
import threading
import time
from copy import deepcopy
from urllib.parse import urlparse
from config_parser import Belt, Sensor, Source, Sink, ControlButton, LedIndicator, TopicComponent, TopicConfig
from physics import PhysicsEngine

import paho.mqtt.client as mqtt


class EditorUI:
    def __init__(self, manager: pygame_gui.UIManager, width: int, height: int, config=None):
        self.manager = manager
        self.width = width
        self.height = height
        self.config = config
        
        # Save Button
        self.save_btn = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(10, 10, 100, 30),
            text='Save Layout',
            manager=self.manager
        )
        
        # Toolbar (Left)
        self.toolbar_panel = pygame_gui.elements.UIPanel(
            relative_rect=pygame.Rect(10, 50, 120, 400),
            manager=self.manager
        )
        self.add_linear_btn = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(10, 10, 100, 30),
            text='+ Linear Belt',
            manager=self.manager,
            container=self.toolbar_panel
        )
        self.add_curved_btn = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(10, 50, 100, 30),
            text='+ Curved Belt',
            manager=self.manager,
            container=self.toolbar_panel
        )
        self.add_sensor_btn = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(10, 90, 100, 30),
            text='+ Sensor',
            manager=self.manager,
            container=self.toolbar_panel
        )
        self.add_button_btn = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(10, 130, 100, 30),
            text='+ Button',
            manager=self.manager,
            container=self.toolbar_panel
        )
        self.add_led_btn = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(10, 170, 100, 30),
            text='+ LED',
            manager=self.manager,
            container=self.toolbar_panel
        )
        self.add_source_btn = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(10, 210, 100, 30),
            text='+ Source',
            manager=self.manager,
            container=self.toolbar_panel
        )
        self.add_sink_btn = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(10, 250, 100, 30),
            text='+ Sink',
            manager=self.manager,
            container=self.toolbar_panel
        )
        self.manager_btn = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(10, 290, 100, 30),
            text='Manager',
            manager=self.manager,
            container=self.toolbar_panel
        )
        self.mqtt_btn = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(10, 330, 100, 30),
            text='MQTT',
            manager=self.manager,
            container=self.toolbar_panel
        )
        
        # Properties Panel (Right)
        self.prop_panel = pygame_gui.elements.UIPanel(
            relative_rect=pygame.Rect(self.width - 250, 0, 250, self.height),
            manager=self.manager
        )

        self.manager_clear_btn = None
        
        self.prop_labels = {}
        self.prop_inputs = {}
        
        # Extended list of all possible properties across all components
        all_props = [
            "id", "x", "y","speed" ,"rotation", "length", "beltWidth", 
            "radius", "startAngle", "endAngle", "directionAngle", 'transition_momentum_duration',
            "width", "height", "sensorType", "laserCount", "beamDistance",
            "interval", "minWidth", "maxWidth", "minHeight", "maxHeight",
            "label", "isOn", "color", "beltShape","beltVertexs"
        ]
        
        # Short display names to keep labels from overflowing
        display_names = {
            "directionAngle": "Direction",
            "beltWidth": "Width",
            "startAngle": "StartAng",
            "endAngle": "EndAng",
            "sensorType": "Type",
            "laserCount": "Beams",
            "beamDistance": "BeamDist",
            "minWidth": "MinW",
            "maxWidth": "MaxW",
            "minHeight": "MinH",
            "maxHeight": "MaxH",
            "beltVertexs":"BeltVertexs",
            "beltShape":"BeltShape",
            "speed":"Speed",
            'transition_momentum_duration':'Trans Dur',
            
            "isOn": "On",
        }
        
        y_offset = 10
        for prop in all_props:
            disp = display_names.get(prop, prop.capitalize())
            lbl = pygame_gui.elements.UILabel(
                relative_rect=pygame.Rect(5, y_offset, 100, 28),
                text=disp + ":",
                manager=self.manager,
                container=self.prop_panel
            )
            if prop == "sensorType":
                inp = pygame_gui.elements.UIDropDownMenu(
                    options_list=["ir", "laser_banner"],
                    starting_option="ir",
                    relative_rect=pygame.Rect(108, y_offset, 127, 28),
                    manager=self.manager,
                    container=self.prop_panel
                )
            elif prop == "beltShape":
                inp = pygame_gui.elements.UIDropDownMenu(
                    options_list=["rectangle", "quadrilateral"],
                    starting_option="rectangle",
                    relative_rect=pygame.Rect(108, y_offset, 127, 28),
                    manager=self.manager,
                    container=self.prop_panel
                )
            else:
                inp = pygame_gui.elements.UITextEntryLine(
                    relative_rect=pygame.Rect(108, y_offset, 127, 28),
                    manager=self.manager,
                    container=self.prop_panel
                )
            self.prop_labels[prop] = lbl
            self.prop_inputs[prop] = inp
            y_offset += 33
            
        self.selected_item = None
        self.hide_all_props()
        
        # Sensor state read-only display (bottom of props panel)
        self.sensor_state_lbl = pygame_gui.elements.UILabel(
            relative_rect=pygame.Rect(5, self.height - 85, 235, 28),
            text="State: --",
            manager=self.manager,
            container=self.prop_panel
        )
        self.sensor_state_lbl.hide()
        self.prop_delete_btn = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(5, self.height - 45, 115, 30),
            text="Delete",
            manager=self.manager,
            container=self.prop_panel
        )
        self.prop_copy_btn = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(125, self.height - 45, 115, 30),
            text="Copy",
            manager=self.manager,
            container=self.prop_panel
        )
        self.prop_delete_btn.hide()
        self.prop_copy_btn.hide()

        self.vertex_labels = []
        self.vertex_x_inputs = []
        self.vertex_y_inputs = []
        vy_offset = 350
        for i in range(4):
            lbl = pygame_gui.elements.UILabel(pygame.Rect(5, vy_offset, 35, 28), f"V{i}:", self.manager, container=self.prop_panel)
            ix = pygame_gui.elements.UITextEntryLine(pygame.Rect(45, vy_offset, 85, 28), self.manager, container=self.prop_panel)
            iy = pygame_gui.elements.UITextEntryLine(pygame.Rect(140, vy_offset, 85, 28), self.manager, container=self.prop_panel)
            self.vertex_labels.append(lbl)
            self.vertex_x_inputs.append(ix)
            self.vertex_y_inputs.append(iy)
            vy_offset += 33
            lbl.hide()
            ix.hide()
            iy.hide()

        self.manager_window = None
        self.manager_list = None
        self.manager_details = None
        self.manager_select_btn = None
        self.manager_duplicate_btn = None
        self.manager_delete_btn = None
        self.manager_refresh_btn = None
        self.manager_items = {}
        self.manager_selected_key = None
        self.manager_clear = False

        self.mqtt_window = None
        self.mqtt_enabled_checkbox = None
        self.mqtt_broker_entry = None
        self.mqtt_prefix_entry = None
        self.mqtt_status_lbl = None
        self.mqtt_topic_list = None
        self.mqtt_topic_id_entry = None
        self.mqtt_topic_entry = None
        self.mqtt_interval_entry = None
        self.mqtt_direction_dd = None
        self.mqtt_format_dd = None
        self.mqtt_endianness_dd = None
        self.mqtt_topic_add_btn = None
        self.mqtt_topic_save_btn = None
        self.mqtt_topic_delete_btn = None
        self.mqtt_component_dd = None
        self.mqtt_component_add_btn = None
        self.mqtt_component_remove_btn = None
        self.mqtt_binding_component_dd = None
        self.mqtt_binding_variable_dd = None
        self.mqtt_binding_entry = None
        self.mqtt_binding_apply_btn = None
        self.mqtt_validate_btn = None
        self.mqtt_debug_topic_entry = None
        self.mqtt_debug_payload_entry = None
        self.mqtt_publish_btn = None
        self.mqtt_subscribe_btn = None
        self.mqtt_disconnect_btn = None
        self.mqtt_composer_table = None
        self.mqtt_composer_preview = None
        self.mqtt_refresh_composer_btn = None
        self.mqtt_decode_btn = None
        self.mqtt_log_box = None
        self.mqtt_topic_items = {}
        self.mqtt_component_options = []
        self.mqtt_selected_topic_id = None
        self.mqtt_debug_client = None
        self.mqtt_debug_connected = False
        self.mqtt_logs = []
        self.mqtt_pending_logs = []
        self.mqtt_pending_decodes = []
        self.ui_thread_id = threading.get_ident()
        self.mqtt_last_payload = None
        self.last_sensor_states = {}

        self.placement_window = None
        self.placement_x_input = None
        self.placement_y_input = None
        self.placement_ok_btn = None
        self.placement_cancel_btn = None
        self.placement_cmd_type = None

    def hide_all_props(self):
        for lbl in self.prop_labels.values():
            lbl.hide()
        for inp in self.prop_inputs.values():
            inp.hide()
        # Guard: sensor_state_lbl is created after hide_all_props is first called in __init__
        lbl = getattr(self, 'sensor_state_lbl', None)
        if lbl:
            lbl.hide()
        delete_btn = getattr(self, 'prop_delete_btn', None)
        if delete_btn:
            delete_btn.hide()
        copy_btn = getattr(self, 'prop_copy_btn', None)
        if copy_btn:
            copy_btn.hide()
        
        for lbl in getattr(self, 'vertex_labels', []):
            lbl.hide()
        for ix in getattr(self, 'vertex_x_inputs', []):
            ix.hide()
        for iy in getattr(self, 'vertex_y_inputs', []):
            iy.hide()

    def show_props_for(self, item):
        self.selected_item = item
        self.hide_all_props()
        if not item:
            return
        self.prop_delete_btn.show()
        self.prop_copy_btn.show()
            
        props_to_show = ["id", "x", "y", "rotation","speed"]
        if isinstance(item, Belt):
            props_to_show.append("directionAngle")
            props_to_show.append("transition_momentum_duration")
            if item.type == "linear":
                props_to_show += ["length", "beltWidth", "beltShape"]
                if getattr(item, "shape", "rectangle") == "quadrilateral":
                    props_to_show.append("beltVertexs")
            elif item.type == "curved":
                props_to_show += ["radius", "beltWidth", "startAngle", "endAngle"]
        elif isinstance(item, Sensor):
            props_to_show += ["width", "height", "sensorType", "label"]
            if item.sensorType == "laser_banner":
                props_to_show += ["laserCount", "beamDistance"]
            # Show the live state label
            self.sensor_state_lbl.set_text("State: --")
            self.sensor_state_lbl.show()
        elif isinstance(item, Source):
            props_to_show = ["id", "x", "y", "rotation","interval", "minWidth", "maxWidth", "minHeight", "maxHeight", "label"]
        elif isinstance(item, Sink):
            props_to_show = ["id", "x", "y", "width", "height", "label"]
        elif isinstance(item, ControlButton):
            props_to_show = ["id", "x", "y", "width", "height", "rotation", "label", "color"]
            self.sensor_state_lbl.set_text("Trigger: --")
            self.sensor_state_lbl.show()
        elif isinstance(item, LedIndicator):
            props_to_show = ["id", "x", "y", "radius", "label", "isOn", "color"]
            
        current_y = 10
        for p in props_to_show:
            if p in self.prop_inputs:
                self.prop_labels[p].set_relative_position((5, current_y))
                self.prop_labels[p].show()
                
                # Map UI property names to model attributes
                attr_name = p
                if p == "beltShape":
                    attr_name = "shape"
                elif p == "beltVertexs":
                    attr_name = "vertexs"
                
                val = getattr(item, attr_name, "")
                
                if p == "beltVertexs":
                    self.prop_inputs[p].set_relative_position((108, current_y))
                    self.prop_inputs[p].show()
                    if not val:
                        # Default points if not set
                        hw = getattr(item, 'length', 2.0) / 2.0
                        hh = getattr(item, 'beltWidth', 1.0) / 2.0
                        val = [{'x': -hw, 'y': -hh}, {'x': hw, 'y': -hh}, {'x': hw, 'y': hh}, {'x': -hw, 'y': hh}]
                    val_str = "; ".join(f"{pt['x']:.2f},{pt['y']:.2f}" for pt in val)
                    self.prop_inputs[p].set_text(val_str)
                    current_y += 33
                    
                    # Also populate the 4 vertex entries
                    for i in range(4):
                        if i < len(val):
                            self.vertex_labels[i].set_relative_position((5, current_y))
                            self.vertex_x_inputs[i].set_relative_position((45, current_y))
                            self.vertex_y_inputs[i].set_relative_position((140, current_y))
                            self.vertex_labels[i].show()
                            self.vertex_x_inputs[i].show()
                            self.vertex_x_inputs[i].set_text(f"{val[i]['x']:.3f}")
                            self.vertex_y_inputs[i].show()
                            self.vertex_y_inputs[i].set_text(f"{val[i]['y']:.3f}")
                            current_y += 33
                        else:
                            self.vertex_labels[i].hide()
                            self.vertex_x_inputs[i].hide()
                            self.vertex_y_inputs[i].hide()
                    continue
                elif isinstance(val, float):
                    val_str = str(round(val, 3))
                else:
                    val_str = str(val)
                
                if p == "sensorType":
                    self.prop_inputs[p].kill()
                    self.prop_inputs[p] = pygame_gui.elements.UIDropDownMenu(
                        options_list=["ir", "laser_banner"],
                        starting_option=val_str,
                        relative_rect=pygame.Rect(108, current_y, 127, 28),
                        manager=self.manager,
                        container=self.prop_panel
                    )
                elif p == "beltShape":
                    self.prop_inputs[p].kill()
                    self.prop_inputs[p] = pygame_gui.elements.UIDropDownMenu(
                        options_list=["rectangle", "quadrilateral"],
                        starting_option=val_str,
                        relative_rect=pygame.Rect(108, current_y, 127, 28),
                        manager=self.manager,
                        container=self.prop_panel
                    )
                else:
                    self.prop_inputs[p].set_relative_position((108, current_y))
                    self.prop_inputs[p].show()
                    self.prop_inputs[p].set_text(val_str)
                    
                current_y += 33
        
        # Position state label dynamically at the end
        if self.sensor_state_lbl:
            self.sensor_state_lbl.set_relative_position((5, current_y + 10))
        self.refresh_button_state()

    def is_ui_blocking_point(self, pos) -> bool:
        panels = [self.prop_panel, self.toolbar_panel, self.save_btn]
        for panel in panels:
            if panel and panel.rect.collidepoint(pos):
                return True
        for window in [self.manager_window, self.mqtt_window]:
            if window and window.alive() and window.rect.collidepoint(pos):
                return True
        return False

    def open_manager_panel(self):
        if not self.config:
            return
        if self.manager_window and self.manager_window.alive():
            self.manager_window.focus()
            self.refresh_manager_panel()
            return

        self.manager_window = pygame_gui.elements.UIWindow(
            rect=pygame.Rect(150, 70, 520, 520),
            manager=self.manager,
            window_display_title="Component Manager",
            resizable=False
        )
        container = self.manager_window.get_container()
        self.manager_list = pygame_gui.elements.UISelectionList(
            relative_rect=pygame.Rect(10, 10, 180, 395),
            item_list=[],
            manager=self.manager,
            container=container
        )
        self.manager_details = pygame_gui.elements.UITextBox(
            html_text="Select a component.",
            relative_rect=pygame.Rect(200, 10, 290, 395),
            manager=self.manager,
            container=container
        )
        self.manager_select_btn = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(10, 420, 90, 32),
            text="Select",
            manager=self.manager,
            container=container
        )
        self.manager_duplicate_btn = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(110, 420, 90, 32),
            text="Duplicate",
            manager=self.manager,
            container=container
        )
        self.manager_delete_btn = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(210, 420, 90, 32),
            text="Delete",
            manager=self.manager,
            container=container
        )
        self.manager_refresh_btn = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(310, 420, 90, 32),
            text="Refresh",
            manager=self.manager,
            container=container
        )
        self.manager_clear_btn = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(410, 420, 90, 32),
            text="Clear",
            manager=self.manager,
            container=container
        )
        self.refresh_manager_panel()

    def refresh_manager_panel(self):
        if not self.manager_list or not self.config:
            return
        self.manager_items = {}
        rows = []
        groups = [
            ("belt", self.config.belts),
            ("sensor", self.config.sensors),
            ("source", self.config.sources),
            ("sink", self.config.sinks),
            ("button", self.config.buttons),
            ("led", self.config.leds),
        ]
        for kind, items in groups:
            for idx, item in enumerate(items):
                key = f"{kind}:{idx}:{item.id}"
                label = f"{kind:<6} {item.id}"
                rows.append(label)
                self.manager_items[label] = (kind, idx, item)
        self.manager_list.set_item_list(rows)
        if self.manager_selected_key not in self.manager_items:
            self.manager_selected_key = None
        self._update_manager_details()

    def _update_manager_details(self):
        if not self.manager_details:
            return
        if not self.manager_selected_key or self.manager_selected_key not in self.manager_items:
            self.manager_details.set_text("Select a component.")
            return
        kind, _, item = self.manager_items[self.manager_selected_key]
        fields = []
        for name, value in item.model_dump(exclude_none=True).items():
            fields.append(f"<b>{name}</b>: {value}")
        self.manager_details.set_text(f"<b>{kind.upper()}</b><br>" + "<br>".join(fields))

    def _collection_for_kind(self, kind: str):
        if kind == "belt":
            return self.config.belts
        if kind == "sensor":
            return self.config.sensors
        if kind == "source":
            return self.config.sources
        if kind == "sink":
            return self.config.sinks
        if kind == "button":
            return self.config.buttons
        if kind == "led":
            return self.config.leds
        return None

    def _make_unique_id(self, kind: str, base_id: str) -> str:
        existing = {
            item.id
            for collection in [self.config.belts, self.config.sensors, self.config.sources, self.config.sinks, self.config.buttons, self.config.leds]
            for item in collection
        }
        import re
        m = re.search(r'(\d+)$', base_id)
        if m:
            num_str = m.group(1)
            prefix = base_id[:-len(num_str)]
            idx = int(num_str) + 1
        else:
            prefix = f"{base_id}_"
            idx = 2
            
        candidate = f"{prefix}{idx}"
        while candidate in existing:
            idx += 1
            candidate = f"{prefix}{idx}"
        return candidate

    def _duplicate_manager_selection(self):
        if not self.manager_selected_key or self.manager_selected_key not in self.manager_items:
            return None
        kind, _, item = self.manager_items[self.manager_selected_key]
        collection = self._collection_for_kind(kind)
        if collection is None:
            return None
        new_item = deepcopy(item)
        new_item.id = self._make_unique_id(kind, item.id)
        if hasattr(new_item, "x"):
            new_item.x += 0.5
        if hasattr(new_item, "y"):
            new_item.y += 0.5
        collection.append(new_item)
        self.manager_selected_key = f"{kind:<6} {new_item.id}"
        self.refresh_manager_panel()
        return new_item

    def _delete_manager_selection(self):
        if not self.manager_selected_key or self.manager_selected_key not in self.manager_items:
            return None
        kind, idx, item = self.manager_items[self.manager_selected_key]
        collection = self._collection_for_kind(kind)
        if collection is not None and item in collection:
            collection.remove(item)
        self.manager_selected_key = None
        self.refresh_manager_panel()
        if self.selected_item == item:
            self.show_props_for(None)
        return item

    def _delete_selected_item(self):
        item = self.selected_item
        if not item:
            return None
        for kind in ["belt", "sensor", "source", "sink", "button", "led"]:
            collection = self._collection_for_kind(kind)
            if collection is not None and item in collection:
                collection.remove(item)
                for topic in self.config.mqttSettings.topics:
                    topic.components = [comp for comp in topic.components if comp.id != item.id]
                self.selected_item = None
                self.show_props_for(None)
                self.refresh_manager_panel()
                self._refresh_composer_table()
                return item
        return None

    def _duplicate_selected_item(self):
        item = self.selected_item
        if not item:
            return None
        for kind in ["belt", "sensor", "source", "sink", "button", "led"]:
            collection = self._collection_for_kind(kind)
            if collection is not None and item in collection:
                new_item = deepcopy(item)
                new_item.id = self._make_unique_id(kind, item.id)
                if hasattr(new_item, "x"):
                    new_item.x += 0.5
                if hasattr(new_item, "y"):
                    new_item.y += 0.5
                collection.append(new_item)
                self.selected_item = new_item
                self.show_props_for(new_item)
                self.refresh_manager_panel()
                self._refresh_composer_table()
                return new_item
        return None

    def open_mqtt_panel(self):
        if not self.config:
            return
        if self.mqtt_window and self.mqtt_window.alive():
            self.mqtt_window.focus()
            self.refresh_mqtt_panel()
            return

        self.mqtt_window = pygame_gui.elements.UIWindow(
            rect=pygame.Rect(430, 45, 740, 720),
            manager=self.manager,
            window_display_title="MQTT Settings",
            resizable=False
        )
        container = self.mqtt_window.get_container()
        settings = self.config.mqttSettings

        self.mqtt_enabled_checkbox = pygame_gui.elements.UICheckBox(
            relative_rect=pygame.Rect(10, 10, 110, 28),
            text="Enabled",
            manager=self.manager,
            container=container,
            initial_state=settings.enabled
        )
        pygame_gui.elements.UILabel(pygame.Rect(130, 10, 80, 28), "Broker:", self.manager, container=container)
        self.mqtt_broker_entry = pygame_gui.elements.UITextEntryLine(
            relative_rect=pygame.Rect(210, 10, 335, 28),
            manager=self.manager,
            container=container,
            initial_text=settings.brokerUrl
        )
        self.mqtt_validate_btn = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(555, 10, 80, 28),
            text="Validate",
            manager=self.manager,
            container=container
        )
        pygame_gui.elements.UILabel(pygame.Rect(10, 45, 80, 28), "Prefix:", self.manager, container=container)
        self.mqtt_prefix_entry = pygame_gui.elements.UITextEntryLine(
            relative_rect=pygame.Rect(90, 45, 210, 28),
            manager=self.manager,
            container=container,
            initial_text=settings.topicPrefix
        )
        self.mqtt_status_lbl = pygame_gui.elements.UILabel(
            relative_rect=pygame.Rect(310, 45, 325, 28),
            text="Status: not checked",
            manager=self.manager,
            container=container
        )
        pygame_gui.elements.UILabel(pygame.Rect(10, 82, 120, 28), "Topics", self.manager, container=container)
        self.mqtt_topic_list = pygame_gui.elements.UISelectionList(
            relative_rect=pygame.Rect(10, 112, 210, 165),
            item_list=[],
            manager=self.manager,
            container=container
        )
        self.mqtt_topic_add_btn = pygame_gui.elements.UIButton(pygame.Rect(10, 285, 65, 30), "New", self.manager, container=container)
        self.mqtt_topic_save_btn = pygame_gui.elements.UIButton(pygame.Rect(82, 285, 65, 30), "Save", self.manager, container=container)
        self.mqtt_topic_delete_btn = pygame_gui.elements.UIButton(pygame.Rect(154, 285, 66, 30), "Delete", self.manager, container=container)

        labels = [("Id:", 235, 112), ("Topic:", 235, 147), ("Interval:", 235, 182)]
        for text, x, y in labels:
            pygame_gui.elements.UILabel(pygame.Rect(x, y, 75, 28), text, self.manager, container=container)
        self.mqtt_topic_id_entry = pygame_gui.elements.UITextEntryLine(pygame.Rect(310, 112, 325, 28), self.manager, container=container)
        self.mqtt_topic_entry = pygame_gui.elements.UITextEntryLine(pygame.Rect(310, 147, 325, 28), self.manager, container=container)
        self.mqtt_interval_entry = pygame_gui.elements.UITextEntryLine(pygame.Rect(310, 182, 80, 28), self.manager, container=container)
        self.mqtt_direction_dd = pygame_gui.elements.UIDropDownMenu(["in", "out"], "out", pygame.Rect(400, 182, 75, 28), self.manager, container=container)
        self.mqtt_format_dd = pygame_gui.elements.UIDropDownMenu(["json", "string", "binary_struct"], "json", pygame.Rect(310, 217, 130, 28), self.manager, container=container)
        self.mqtt_endianness_dd = pygame_gui.elements.UIDropDownMenu(["LE", "BE"], "LE", pygame.Rect(450, 217, 75, 28), self.manager, container=container)

        pygame_gui.elements.UILabel(pygame.Rect(235, 260, 150, 28), "Topic components:", self.manager, container=container)
        self.mqtt_component_dd = pygame_gui.elements.UIDropDownMenu(["none"], "none", pygame.Rect(235, 290, 220, 28), self.manager, container=container)
        self.mqtt_component_add_btn = pygame_gui.elements.UIButton(pygame.Rect(465, 290, 65, 28), "Add", self.manager, container=container)
        self.mqtt_component_remove_btn = pygame_gui.elements.UIButton(pygame.Rect(540, 290, 95, 28), "Remove", self.manager, container=container)

        pygame_gui.elements.UILabel(pygame.Rect(10, 330, 170, 28), "Component binding", self.manager, container=container)
        self.mqtt_binding_component_dd = pygame_gui.elements.UIDropDownMenu(["none"], "none", pygame.Rect(10, 365, 210, 28), self.manager, container=container)
        self.mqtt_binding_variable_dd = pygame_gui.elements.UIDropDownMenu(["stateBinding"], "stateBinding", pygame.Rect(230, 365, 155, 28), self.manager, container=container)
        self.mqtt_binding_entry = pygame_gui.elements.UITextEntryLine(
            pygame.Rect(395, 365, 235, 28),
            self.manager,
            container=container,
            placeholder_text="MQTT variable or PLC tag"
        )
        self.mqtt_binding_apply_btn = pygame_gui.elements.UIButton(pygame.Rect(640, 365, 65, 28), "Apply", self.manager, container=container)

        pygame_gui.elements.UILabel(pygame.Rect(10, 405, 160, 28), "Debug topic:", self.manager, container=container)
        self.mqtt_debug_topic_entry = pygame_gui.elements.UITextEntryLine(
            pygame.Rect(105, 405, 290, 28),
            self.manager,
            container=container,
            initial_text=self._full_topic(self._selected_topic()) or "sim/test_topic"
        )
        self.mqtt_publish_btn = pygame_gui.elements.UIButton(pygame.Rect(405, 405, 80, 28), "Publish", self.manager, container=container)
        self.mqtt_subscribe_btn = pygame_gui.elements.UIButton(pygame.Rect(493, 405, 90, 28), "Subscribe", self.manager, container=container)
        self.mqtt_disconnect_btn = pygame_gui.elements.UIButton(pygame.Rect(590, 405, 90, 28), "Stop Sub", self.manager, container=container)
        self.mqtt_debug_payload_entry = pygame_gui.elements.UITextEntryLine(
            pygame.Rect(105, 438, 575, 28),
            self.manager,
            container=container,
            placeholder_text="Optional debug payload; blank uses composer output"
        )

        pygame_gui.elements.UILabel(pygame.Rect(10, 475, 160, 28), "Composer table", self.manager, container=container)
        self.mqtt_refresh_composer_btn = pygame_gui.elements.UIButton(pygame.Rect(580, 475, 100, 28), "Refresh", self.manager, container=container)
        self.mqtt_composer_table = pygame_gui.elements.UISelectionList(
            relative_rect=pygame.Rect(10, 508, 380, 120),
            item_list=[],
            manager=self.manager,
            container=container,
            allow_multi_select=False
        )
        self.mqtt_composer_preview = pygame_gui.elements.UITextBox(
            html_text="Payload preview",
            relative_rect=pygame.Rect(400, 508, 280, 120),
            manager=self.manager,
            container=container
        )
        self.mqtt_decode_btn = pygame_gui.elements.UIButton(pygame.Rect(400, 475, 170, 28), "Decode Last Rx", self.manager, container=container)
        self.mqtt_log_box = pygame_gui.elements.UITextBox(
            html_text="MQTT log",
            relative_rect=pygame.Rect(10, 635, 670, 50),
            manager=self.manager,
            container=container
        )

        self.refresh_mqtt_panel()

    def _component_options(self):
        options = []
        for kind, items in [
            ("belt", self.config.belts),
            ("sensor", self.config.sensors),
            ("source", self.config.sources),
            ("sink", self.config.sinks),
            ("button", self.config.buttons),
            ("led", self.config.leds),
        ]:
            for item in items:
                options.append(f"{kind}:{item.id}")
        return options or ["none"]

    def _topic_component_options(self):
        options = []
        for kind, items in [
            ("belt", self.config.belts),
            ("sensor", self.config.sensors),
            ("button", self.config.buttons),
            ("led", self.config.leds),
        ]:
            for item in items:
                options.append(f"{kind}:{item.id}")
        return options or ["none"]

    def _selected_option_value(self, dropdown):
        option = dropdown.selected_option if dropdown else ""
        return option[0] if isinstance(option, tuple) else option

    def _mqtt_log(self, message: str):
        if threading.get_ident() != self.ui_thread_id:
            self.mqtt_pending_logs.append(message)
            return
        stamp = time.strftime("%H:%M:%S")
        self.mqtt_logs.append(f"[{stamp}] {message}")
        self.mqtt_logs = self.mqtt_logs[-6:]
        if self.mqtt_log_box:
            safe_lines = [line.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;") for line in self.mqtt_logs]
            self.mqtt_log_box.set_text("<br>".join(safe_lines) or "MQTT log")

    def _flush_mqtt_logs(self):
        pending = self.mqtt_pending_logs
        self.mqtt_pending_logs = []
        for message in pending:
            self._mqtt_log(message)

    def _flush_mqtt_decodes(self):
        pending = self.mqtt_pending_decodes
        self.mqtt_pending_decodes = []
        for decoded in pending:
            self._apply_decoded_values(decoded)
            self._mqtt_log(f"Decoded: {decoded}")
        if pending:
            self.refresh_manager_panel()
            self._refresh_composer_table()

    def _full_topic(self, topic):
        if topic is None:
            return ""
        prefix = self.config.mqttSettings.topicPrefix.strip("/")
        name = topic.topic.strip("/")
        if prefix and name:
            return f"{prefix}/{name}"
        return prefix or name

    def _parse_broker_url(self):
        self._apply_mqtt_settings()
        url = urlparse(self.config.mqttSettings.brokerUrl)
        scheme = url.scheme.lower()
        if scheme == "tcp":
            scheme = "mqtt"
        if scheme not in {"mqtt", "mqtts", "ws", "wss"} or not url.hostname:
            raise ValueError("Use mqtt://host:1883 or ws://host:9001/mqtt")
        default_port = 8883 if scheme in {"mqtts", "wss"} else 1883
        if scheme in {"ws", "wss"} and not url.port:
            default_port = 443 if scheme == "wss" else 9001
        return url, scheme, url.hostname, url.port or default_port

    def _create_mqtt_client(self, on_message=None):
        if mqtt is None:
            raise RuntimeError("paho-mqtt is not installed")
        url, scheme, host, port = self._parse_broker_url()
        transport = "websockets" if scheme in {"ws", "wss"} else "tcp"
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, transport=transport)
        if scheme in {"mqtts", "wss"}:
            client.tls_set()
        if transport == "websockets":
            client.ws_set_options(path=url.path or "/mqtt")
        if on_message:
            client.on_message = on_message
        return client, host, port

    def _component_kind_and_item(self, comp):
        kind = "sensor" if comp.type in {"sensor", "laser_banner"} else comp.type
        collection = self._collection_for_kind(kind)
        if collection is None:
            return None, None
        for item in collection:
            if item.id == comp.id:
                return kind, item
        return None, None

    def _composer_rows(self, topic=None):
        topic = topic or self._selected_topic()
        if not topic:
            return []
        rows = []
        for comp in topic.components:
            kind, item = self._component_kind_and_item(comp)
            if not item:
                rows.append({"component": comp.id, "type": comp.type, "field": "missing", "binding": "", "value": None})
                continue
            if kind == "sensor":
                state = self.last_sensor_states.get(item.id, getattr(item, "isActive", False))
                rows.append({"component": item.id, "type": comp.type, "field": "state", "binding": getattr(item, "stateBinding", "") or "", "value": state})
                if getattr(item, "sensorType", "") == "laser_banner":
                    rows.append({"component": item.id, "type": comp.type, "field": "word", "binding": getattr(item, "wordBinding", "") or "", "value": getattr(item, "detectedWord", 0)})
            elif kind == "belt":
                rows.append({"component": item.id, "type": comp.type, "field": "speed", "binding": getattr(item, "speedBinding", "") or "", "value": getattr(item, "speed", 0.0)})
                rows.append({"component": item.id, "type": comp.type, "field": "directionAngle", "binding": getattr(item, "directionAngleBinding", "") or "", "value": getattr(item, "directionAngle", 0.0)})
                rows.append({"component": item.id, "type": comp.type, "field": "direction", "binding": getattr(item, "directionBinding", "") or "", "value": getattr(item, "direction", 1)})
            elif kind == "button":
                rows.append({"component": item.id, "type": comp.type, "field": "state", "binding": getattr(item, "stateBinding", "") or "", "value": getattr(item, "sensor_states", False)})
            elif kind == "led":
                rows.append({"component": item.id, "type": comp.type, "field": "state", "binding": getattr(item, "stateBinding", "") or "", "value": getattr(item, "isOn", False)})
        return rows

    def _composer_values(self, topic=None):
        data = {}
        for row in self._composer_rows(topic):
            component_data = data.setdefault(row["component"], {})
            key = row["binding"] or row["field"]
            component_data[key] = row["value"]
        return data

    def _encode_topic_payload(self, topic=None):
        topic = topic or self._selected_topic()
        if not topic:
            return b""
        values = self._composer_values(topic)
        fmt = topic.format
        if fmt == "json":
            return json.dumps(values, separators=(",", ":")).encode("utf-8")
        if fmt in {"raw", "string"}:
            parts = []
            for component, fields in values.items():
                for field, value in fields.items():
                    parts.append(f"{component}.{field}={value}")
            return ";".join(parts).encode("utf-8")
        endian = "<" if topic.endianness == "LE" else ">"
        payload = bytearray()
        for row in self._composer_rows(topic):
            value = row["value"]
            if isinstance(value, bool):
                payload.extend(struct.pack(endian + "?", value))
            elif isinstance(value, int):
                payload.extend(struct.pack(endian + "i", value))
            elif isinstance(value, float):
                payload.extend(struct.pack(endian + "f", value))
            else:
                encoded = str(value).encode("utf-8")[:32]
                payload.extend(struct.pack(endian + "H", len(encoded)))
                payload.extend(encoded)
        return bytes(payload)

    def _estimate_topic_bandwidth(self, topic=None):
        topic = topic or self._selected_topic()
        if not topic:
            return {"payload_bytes": 0, "estimated_packet_bytes": 0, "messages_per_second": 0.0, "bytes_per_second": 0.0}
        payload = self._encode_topic_payload(topic)
        topic_bytes = len(self._full_topic(topic).encode("utf-8"))
        # MQTT QoS 0 publish framing is roughly fixed header + topic length + topic + payload.
        estimated_packet_bytes = 4 + topic_bytes + len(payload)
        messages_per_second = 1000.0 / max(1, topic.interval)
        return {
            "payload_bytes": len(payload),
            "estimated_packet_bytes": estimated_packet_bytes,
            "messages_per_second": messages_per_second,
            "bytes_per_second": estimated_packet_bytes * messages_per_second,
        }

    def _decode_topic_payload(self, payload, topic=None):
        topic = topic or self._selected_topic()
        if not topic:
            return None
        if isinstance(payload, str):
            payload_bytes = payload.encode("utf-8")
        else:
            payload_bytes = payload
        if topic.format == "json":
            return json.loads(payload_bytes.decode("utf-8"))
        if topic.format in {"raw", "string"}:
            text = payload_bytes.decode("utf-8", errors="replace")
            decoded = {}
            for part in text.split(";"):
                if "=" not in part:
                    continue
                left, value = part.split("=", 1)
                component, field = left.split(".", 1) if "." in left else ("message", left)
                decoded.setdefault(component, {})[field] = value
            return decoded or {"message": text}
        endian = "<" if topic.endianness == "LE" else ">"
        offset = 0
        decoded = {}
        for row in self._composer_rows(topic):
            current = row["value"]
            try:
                if isinstance(current, bool):
                    size = struct.calcsize("?")
                    value = struct.unpack_from(endian + "?", payload_bytes, offset)[0]
                elif isinstance(current, int):
                    size = struct.calcsize("i")
                    value = struct.unpack_from(endian + "i", payload_bytes, offset)[0]
                elif isinstance(current, float):
                    size = struct.calcsize("f")
                    value = struct.unpack_from(endian + "f", payload_bytes, offset)[0]
                else:
                    str_len = struct.unpack_from(endian + "H", payload_bytes, offset)[0]
                    offset += struct.calcsize("H")
                    value = payload_bytes[offset:offset + str_len].decode("utf-8", errors="replace")
                    size = str_len
                offset += size
            except (struct.error, UnicodeDecodeError):
                value = None
            decoded.setdefault(row["component"], {})[row["binding"] or row["field"]] = value
        return decoded

    def _apply_decoded_values(self, decoded):
        if not isinstance(decoded, dict):
            return
        for component_id, fields in decoded.items():
            if not isinstance(fields, dict):
                continue
            item = None
            for collection in [self.config.belts, self.config.sensors, self.config.sources, self.config.sinks, self.config.buttons, self.config.leds]:
                item = next((candidate for candidate in collection if candidate.id == component_id), None)
                if item:
                    break
            if not item:
                continue
            for key, value in fields.items():
                binding_attrs = ["speedBinding", "directionAngleBinding", "directionBinding", "stateBinding", "wordBinding", "triggerBinding", "intervalBinding"]
                attr = next((name[:-7] for name in binding_attrs if getattr(item, name, None) == key), key)
                if isinstance(item, Sensor) and attr == "state":
                    attr = "isActive"
                elif isinstance(item, Sensor) and attr == "word":
                    attr = "detectedWord"
                elif isinstance(item, ControlButton) and attr == "state":
                    attr = "sensor_states"
                elif isinstance(item, LedIndicator) and attr == "state":
                    attr = "isOn"
                if hasattr(item, attr):
                    current = getattr(item, attr)
                    try:
                        if isinstance(current, bool):
                            value = str(value).lower() in {"1", "true", "yes", "on"}
                        elif isinstance(current, int):
                            value = int(float(value))
                        elif isinstance(current, float):
                            value = float(value)
                    except (TypeError, ValueError):
                        continue
                    setattr(item, attr, value)

    def _refresh_composer_table(self):
        if not self.mqtt_composer_table:
            return
        rows = self._composer_rows()
        labels = ["Component        Field              Binding           Value"]
        for row in rows:
            value = str(row["value"])
            labels.append(f"{row['component'][:15]:<16} {row['field'][:17]:<18} {(row['binding'] or '-')[:16]:<17} {value[:24]}")
        self.mqtt_composer_table.set_item_list(labels)
        try:
            payload = self._encode_topic_payload()
            bandwidth = self._estimate_topic_bandwidth()
            preview = payload.decode("utf-8") if self._selected_topic() and self._selected_topic().format != "binary_struct" else payload.hex(" ")
        except Exception as exc:
            preview = f"Encode error: {exc}"
            bandwidth = None
        safe = preview.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
        if self.mqtt_composer_preview:
            if bandwidth:
                bw_text = f"<br><br>{bandwidth['payload_bytes']} payload bytes; {bandwidth['bytes_per_second']:.1f} B/s est."
            else:
                bw_text = ""
            self.mqtt_composer_preview.set_text(f"<b>Payload preview</b><br>{safe}{bw_text}")

    def _debug_publish(self):
        topic_name = self.mqtt_debug_topic_entry.get_text().strip()
        if not topic_name:
            self._mqtt_log("Publish skipped: empty topic")
            return
        payload_text = self.mqtt_debug_payload_entry.get_text()
        payload = payload_text.encode("utf-8") if payload_text else self._encode_topic_payload()
        try:
            client, host, port = self._create_mqtt_client()
            client.connect(host, port, 30)
            client.loop_start()
            result = client.publish(topic_name, payload)
            result.wait_for_publish(timeout=3)
            client.loop_stop()
            client.disconnect()
            shown = payload.hex(" ") if isinstance(payload, bytes) and b"\x00" in payload else payload.decode("utf-8", errors="replace")
            self._mqtt_log(f"Published {len(payload)} bytes to {topic_name}: {shown[:80]}")
        except Exception as exc:
            self._mqtt_log(f"Publish failed: {exc}")

    def _debug_subscribe(self):
        topic_name = self.mqtt_debug_topic_entry.get_text().strip()
        if not topic_name:
            self._mqtt_log("Subscribe skipped: empty topic")
            return
        self._debug_disconnect()

        def on_connect(client, userdata, flags, reason_code, properties=None):
            self.mqtt_debug_connected = True
            client.subscribe(topic_name)
            self._mqtt_log(f"Subscribed to {topic_name} ({reason_code})")

        def on_message(client, userdata, msg):
            self.mqtt_last_payload = msg.payload
            text = msg.payload.decode("utf-8", errors="replace")
            self._mqtt_log(f"RX {msg.topic}: {text[:100]}")
            topic = self._selected_topic()
            if topic and msg.topic == self._full_topic(topic):
                try:
                    decoded = self._decode_topic_payload(msg.payload, topic)
                    self.mqtt_pending_decodes.append(decoded)
                except Exception as exc:
                    self._mqtt_log(f"Decode failed: {exc}")

        try:
            client, host, port = self._create_mqtt_client(on_message=on_message)
            client.on_connect = on_connect
            client.connect(host, port, 30)
            client.loop_start()
            self.mqtt_debug_client = client
            self._mqtt_log(f"Connecting subscriber to {host}:{port}")
        except Exception as exc:
            self.mqtt_debug_client = None
            self._mqtt_log(f"Subscribe failed: {exc}")

    def _debug_disconnect(self):
        if not self.mqtt_debug_client:
            return
        try:
            self.mqtt_debug_client.loop_stop()
            self.mqtt_debug_client.disconnect()
            self._mqtt_log("Subscriber stopped")
        except Exception as exc:
            self._mqtt_log(f"Disconnect failed: {exc}")
        self.mqtt_debug_client = None
        self.mqtt_debug_connected = False

    def _decode_last_rx(self):
        if not self.mqtt_last_payload:
            self._mqtt_log("No received payload to decode")
            return
        try:
            decoded = self._decode_topic_payload(self.mqtt_last_payload)
            self._apply_decoded_values(decoded)
            self._mqtt_log(f"Decoded last RX: {decoded}")
            self.refresh_manager_panel()
            self._refresh_composer_table()
        except Exception as exc:
            self._mqtt_log(f"Decode failed: {exc}")

    def refresh_mqtt_panel(self):
        if not self.mqtt_window or not self.mqtt_window.alive() or not self.config:
            return
        settings = self.config.mqttSettings
        self.mqtt_topic_items = {}
        rows = []
        for topic in settings.topics:
            label = f"{topic.id}  {topic.direction}  {topic.topic}"
            rows.append(label)
            self.mqtt_topic_items[label] = topic
        self.mqtt_topic_list.set_item_list(rows)

        options = self._component_options()
        topic_options = self._topic_component_options()
        self.mqtt_component_options = topic_options
        self._replace_dropdown("mqtt_component_dd", topic_options, topic_options[0], pygame.Rect(235, 290, 220, 28))
        self._replace_dropdown("mqtt_binding_component_dd", options, options[0], pygame.Rect(10, 365, 210, 28))
        self._refresh_binding_variable_dropdown()

        selected = self._selected_topic()
        self._show_topic(selected)
        if self.mqtt_debug_topic_entry:
            self.mqtt_debug_topic_entry.set_text(self._full_topic(selected) or self.mqtt_debug_topic_entry.get_text() or "sim/test_topic")
        self._refresh_composer_table()

    def _replace_dropdown(self, attr_name, options, selected, rect):
        old = getattr(self, attr_name)
        container = self.mqtt_window.get_container()
        if old:
            old.kill()
        setattr(
            self,
            attr_name,
            pygame_gui.elements.UIDropDownMenu(options, selected, rect, self.manager, container=container)
        )

    def _selected_topic(self):
        if not self.mqtt_selected_topic_id:
            return None
        for topic in self.config.mqttSettings.topics:
            if topic.id == self.mqtt_selected_topic_id:
                return topic
        return None

    def _show_topic(self, topic):
        if not self.mqtt_topic_id_entry:
            return
        if not topic:
            self.mqtt_topic_id_entry.set_text("")
            self.mqtt_topic_entry.set_text("")
            self.mqtt_interval_entry.set_text("1000")
            return
        self.mqtt_topic_id_entry.set_text(topic.id)
        self.mqtt_topic_entry.set_text(topic.topic)
        self.mqtt_interval_entry.set_text(str(topic.interval))
        fmt = "string" if topic.format == "raw" else topic.format
        self._replace_dropdown("mqtt_direction_dd", ["in", "out"], topic.direction, pygame.Rect(400, 182, 75, 28))
        self._replace_dropdown("mqtt_format_dd", ["json", "string", "binary_struct"], fmt, pygame.Rect(310, 217, 130, 28))
        self._replace_dropdown("mqtt_endianness_dd", ["LE", "BE"], topic.endianness, pygame.Rect(450, 217, 75, 28))

    def _new_topic_id(self):
        existing = {topic.id for topic in self.config.mqttSettings.topics}
        idx = len(existing) + 1
        candidate = f"topic_{idx}"
        while candidate in existing:
            idx += 1
            candidate = f"topic_{idx}"
        return candidate

    def _save_topic_from_form(self):
        settings = self.config.mqttSettings
        topic_id = self.mqtt_topic_id_entry.get_text().strip() or self._new_topic_id()
        topic_name = self.mqtt_topic_entry.get_text().strip() or topic_id
        try:
            interval = max(1, int(float(self.mqtt_interval_entry.get_text())))
        except ValueError:
            interval = 1000
        topic = self._selected_topic()
        if topic is None:
            topic = TopicConfig(id=topic_id, topic=topic_name, interval=interval, direction="out", format="json", endianness="LE")
            settings.topics.append(topic)
        topic.id = topic_id
        topic.topic = topic_name
        topic.interval = interval
        topic.direction = self.mqtt_direction_dd.selected_option[0] if isinstance(self.mqtt_direction_dd.selected_option, tuple) else self.mqtt_direction_dd.selected_option
        topic.format = self.mqtt_format_dd.selected_option[0] if isinstance(self.mqtt_format_dd.selected_option, tuple) else self.mqtt_format_dd.selected_option
        topic.endianness = self.mqtt_endianness_dd.selected_option[0] if isinstance(self.mqtt_endianness_dd.selected_option, tuple) else self.mqtt_endianness_dd.selected_option
        self.mqtt_selected_topic_id = topic.id
        self.refresh_mqtt_panel()

    def _find_component_from_option(self, option):
        if not option or option == "none" or ":" not in option:
            return None, None
        kind, item_id = option.split(":", 1)
        collection = self._collection_for_kind(kind)
        if collection is None:
            return None, None
        for item in collection:
            if item.id == item_id:
                return kind, item
        return None, None

    def _refresh_binding_variable_dropdown(self):
        option = self.mqtt_binding_component_dd.selected_option if self.mqtt_binding_component_dd else "none"
        if isinstance(option, tuple):
            option = option[0]
        kind, _ = self._find_component_from_option(option)
        if kind == "belt":
            vars_ = ["speedBinding", "directionAngleBinding", "directionBinding"]
        elif kind == "sensor":
            vars_ = ["stateBinding", "wordBinding"]
        elif kind == "source":
            vars_ = ["triggerBinding", "intervalBinding"]
        elif kind in {"button", "led"}:
            vars_ = ["stateBinding"]
        else:
            vars_ = ["id"]
        self._replace_dropdown("mqtt_binding_variable_dd", vars_, vars_[0], pygame.Rect(230, 365, 155, 28))

    def _apply_mqtt_settings(self):
        if not self.mqtt_broker_entry or not self.mqtt_prefix_entry:
            return
        settings = self.config.mqttSettings
        settings.brokerUrl = self.mqtt_broker_entry.get_text().strip() or settings.brokerUrl
        settings.topicPrefix = self.mqtt_prefix_entry.get_text().strip()

    def _validate_mqtt_connection(self):
        self._apply_mqtt_settings()
        url = urlparse(self.config.mqttSettings.brokerUrl)
        scheme = url.scheme.lower()
        if scheme not in {"mqtt", "mqtts", "ws", "wss", "tcp"} or not url.hostname:
            self.mqtt_status_lbl.set_text("Status: invalid URL")
            return
        default_port = 8883 if scheme in {"mqtts", "wss"} else 1883
        if scheme == "ws":
            default_port = 9001
        elif scheme == "wss":
            default_port = 443
        port = url.port or default_port
        try:
            with socket.create_connection((url.hostname, port), timeout=2.0):
                self.mqtt_status_lbl.set_text("Status: TCP reachable")
        except OSError as exc:
            self.mqtt_status_lbl.set_text(f"Status: {str(exc)[:24]}")

    def _add_component_to_topic(self):
        topic = self._selected_topic()
        if topic is None:
            self._save_topic_from_form()
            topic = self._selected_topic()
        option = self.mqtt_component_dd.selected_option
        if isinstance(option, tuple):
            option = option[0]
        kind, item = self._find_component_from_option(option)
        if not topic or not item or kind not in {"belt", "sensor", "button", "led"}:
            return
        component_type = "laser_banner" if kind == "sensor" and getattr(item, "sensorType", "") == "laser_banner" else kind
        if not any(comp.id == item.id and comp.type == component_type for comp in topic.components):
            topic.components.append(TopicComponent(id=item.id, type=component_type))
            self._mqtt_log(f"Added {item.id} to {topic.id}")
        self._refresh_composer_table()

    def _remove_component_from_topic(self):
        topic = self._selected_topic()
        option = self.mqtt_component_dd.selected_option
        if isinstance(option, tuple):
            option = option[0]
        kind, item = self._find_component_from_option(option)
        if not topic or not item:
            return
        topic.components = [comp for comp in topic.components if comp.id != item.id]
        self._mqtt_log(f"Removed {item.id} from {topic.id}")
        self._refresh_composer_table()

    def _apply_binding_from_form(self):
        option = self.mqtt_binding_component_dd.selected_option
        if isinstance(option, tuple):
            option = option[0]
        _, item = self._find_component_from_option(option)
        variable = self.mqtt_binding_variable_dd.selected_option
        if isinstance(variable, tuple):
            variable = variable[0]
        if item and hasattr(item, variable):
            setattr(item, variable, self.mqtt_binding_entry.get_text().strip() or None)
            self._refresh_composer_table()

    def handle_resize(self, width: int, height: int):
        self.width = width
        self.height = height
        self.toolbar_panel.set_dimensions((120, height))
        self.prop_panel.set_position((width - 250, 0))
        self.prop_panel.set_dimensions((250, height))
        # Update delete/copy buttons to anchor to bottom
        if getattr(self, 'prop_delete_btn', None):
            self.prop_delete_btn.set_relative_position((5, self.height - 45))
        if getattr(self, 'prop_copy_btn', None):
            self.prop_copy_btn.set_relative_position((125, self.height - 45))

    def handle_event(self, event) -> str:
        """Returns commands or None."""
        if event.type == pygame_gui.UI_BUTTON_PRESSED:
            if event.ui_element == self.save_btn:
                return "SAVE_DIALOG"
            elif event.ui_element == self.add_linear_btn:
                self.open_placement_dialog("ADD_LINEAR")
            elif event.ui_element == self.add_curved_btn:
                self.open_placement_dialog("ADD_CURVED")
            elif event.ui_element == self.add_sensor_btn:
                self.open_placement_dialog("ADD_SENSOR")
            elif event.ui_element == self.add_button_btn:
                self.open_placement_dialog("ADD_BUTTON")
            elif event.ui_element == self.add_led_btn:
                self.open_placement_dialog("ADD_LED")
            elif event.ui_element == self.add_source_btn:
                self.open_placement_dialog("ADD_SOURCE")
            elif event.ui_element == self.add_sink_btn:
                self.open_placement_dialog("ADD_SINK")
            elif getattr(self, 'placement_cancel_btn', None) and event.ui_element == self.placement_cancel_btn:
                if self.placement_window:
                    self.placement_window.kill()
                    self.placement_window = None
            elif getattr(self, 'placement_ok_btn', None) and event.ui_element == self.placement_ok_btn:
                try:
                    x = float(self.placement_x_input.get_text())
                    y = float(self.placement_y_input.get_text())
                    cmd = ("PLACE_COMPONENT", self.placement_cmd_type, x, y)
                    if self.placement_window:
                        self.placement_window.kill()
                        self.placement_window = None
                    return cmd
                except ValueError:
                    pass # ignore invalid
            elif event.ui_element == self.prop_delete_btn:
                deleted = self._delete_selected_item()
                if deleted:
                    return ("SELECT_ITEM", None)
            elif event.ui_element == self.prop_copy_btn:
                copied = self._duplicate_selected_item()
                if copied:
                    return ("SELECT_ITEM", copied)
            elif event.ui_element == self.manager_btn:
                return "OPEN_MANAGER"
            elif event.ui_element == self.mqtt_btn:
                return "OPEN_MQTT"
            elif event.ui_element == self.manager_select_btn:
                if self.manager_selected_key in self.manager_items:
                    _, _, item = self.manager_items[self.manager_selected_key]
                    self.show_props_for(item)
                    return ("SELECT_ITEM", item)
            elif event.ui_element == self.manager_duplicate_btn:
                item = self._duplicate_manager_selection()
                if item:
                    self.show_props_for(item)
                    return ("SELECT_ITEM", item)
            elif event.ui_element == self.manager_delete_btn:
                deleted = self._delete_manager_selection()
                if deleted and deleted == self.selected_item:
                    return ("SELECT_ITEM", None)
            elif event.ui_element == self.manager_refresh_btn:
                self.refresh_manager_panel()
            elif self.manager_clear_btn and event.ui_element == self.manager_clear_btn:
                self.manager_clear = True
                
            elif event.ui_element == self.mqtt_topic_add_btn:
                topic_id = self._new_topic_id()
                topic = TopicConfig(id=topic_id, topic=topic_id, interval=1000, direction="out", format="json", endianness="LE")
                self.config.mqttSettings.topics.append(topic)
                self.mqtt_selected_topic_id = topic.id
                self.refresh_mqtt_panel()
            elif event.ui_element == self.mqtt_topic_save_btn:
                self._apply_mqtt_settings()
                self._save_topic_from_form()
            elif event.ui_element == self.mqtt_topic_delete_btn:
                topic = self._selected_topic()
                if topic:
                    self.config.mqttSettings.topics.remove(topic)
                    self.mqtt_selected_topic_id = None
                    self.refresh_mqtt_panel()
            elif event.ui_element == self.mqtt_component_add_btn:
                self._add_component_to_topic()
            elif event.ui_element == self.mqtt_component_remove_btn:
                self._remove_component_from_topic()
            elif event.ui_element == self.mqtt_binding_apply_btn:
                self._apply_binding_from_form()
                self.refresh_manager_panel()
            elif event.ui_element == self.mqtt_validate_btn:
                self._validate_mqtt_connection()
            elif event.ui_element == self.mqtt_publish_btn:
                self._debug_publish()
            elif event.ui_element == self.mqtt_subscribe_btn:
                self._debug_subscribe()
            elif event.ui_element == self.mqtt_disconnect_btn:
                self._debug_disconnect()
            elif event.ui_element == self.mqtt_refresh_composer_btn:
                self._refresh_composer_table()
            elif event.ui_element == self.mqtt_decode_btn:
                self._decode_last_rx()

        if event.type == pygame_gui.UI_SELECTION_LIST_NEW_SELECTION:
            if event.ui_element == self.manager_list:
                self.manager_selected_key = event.text
                self._update_manager_details()
            elif event.ui_element == self.mqtt_topic_list:
                topic = self.mqtt_topic_items.get(event.text)
                self.mqtt_selected_topic_id = topic.id if topic else None
                self._show_topic(topic)
                if self.mqtt_debug_topic_entry:
                    self.mqtt_debug_topic_entry.set_text(self._full_topic(topic) or "sim/test_topic")
                self._refresh_composer_table()

        if event.type == pygame_gui.UI_CHECK_BOX_CHECKED:
            if event.ui_element == self.mqtt_enabled_checkbox:
                self.config.mqttSettings.enabled = True

        if event.type == pygame_gui.UI_CHECK_BOX_UNCHECKED:
            if event.ui_element == self.mqtt_enabled_checkbox:
                self.config.mqttSettings.enabled = False

        if event.type == pygame_gui.UI_WINDOW_CLOSE:
            if event.ui_element == self.manager_window:
                self.manager_window = None
            elif event.ui_element == self.mqtt_window:
                self._debug_disconnect()
                self._apply_mqtt_settings()
                self.mqtt_window = None
                
        if event.type == pygame_gui.UI_DROP_DOWN_MENU_CHANGED and self.selected_item:
            if event.ui_element == self.prop_inputs.get("sensorType"):
                val_str = event.text
                setattr(self.selected_item, "sensorType", val_str)
                self.show_props_for(self.selected_item)
            elif event.ui_element == self.prop_inputs.get("beltShape"):
                val_str = event.text
                setattr(self.selected_item, "shape", val_str)
                # Auto-initialize default vertices if switched to quadrilateral
                if val_str == "quadrilateral" and not getattr(self.selected_item, "vertexs", None):
                    hw = getattr(self.selected_item, 'length', 2.0) / 2.0
                    hh = getattr(self.selected_item, 'beltWidth', 1.0) / 2.0
                    self.selected_item.vertexs = [
                        {'x': -hw, 'y': -hh}, {'x': hw, 'y': -hh}, {'x': hw, 'y': hh}, {'x': -hw, 'y': hh}
                    ]
                self.show_props_for(self.selected_item)

        if event.type == pygame_gui.UI_DROP_DOWN_MENU_CHANGED:
            if event.ui_element == self.mqtt_binding_component_dd:
                self._refresh_binding_variable_dropdown()
            elif event.ui_element == self.mqtt_binding_variable_dd:
                option = self.mqtt_binding_component_dd.selected_option
                if isinstance(option, tuple):
                    option = option[0]
                _, item = self._find_component_from_option(option)
                variable = event.text
                if item and hasattr(item, variable):
                    self.mqtt_binding_entry.set_text(str(getattr(item, variable) or ""))
            elif event.ui_element in [self.mqtt_direction_dd, self.mqtt_format_dd, self.mqtt_endianness_dd]:
                self._save_topic_from_form()
                self._refresh_composer_table()
                
        if event.type == pygame_gui.UI_TEXT_ENTRY_CHANGED and self.selected_item:
            # Find which input changed
            for prop, inp in self.prop_inputs.items():
                if event.ui_element == inp:
                    val_str = inp.get_text()
                    try:
                        attr_name = prop
                        if prop == "beltShape":
                            attr_name = "shape"
                        elif prop == "beltVertexs":
                            attr_name = "vertexs"
                        
                        if attr_name == "vertexs":
                            # Parse string of format "x0,y0; x1,y1; x2,y2; x3,y3"
                            parts = val_str.split(';')
                            pts = []
                            for part in parts:
                                part = part.strip()
                                if not part: continue
                                coords = part.split(',')
                                if len(coords) == 2:
                                    pts.append({'x': float(coords[0]), 'y': float(coords[1])})
                            if len(pts) == 4:
                                setattr(self.selected_item, attr_name, pts)
                        else:
                            existing_val = getattr(self.selected_item, attr_name, "")
                            if isinstance(existing_val, bool):
                                setattr(self.selected_item, attr_name, val_str.strip().lower() in {"1", "true", "yes", "on", "pressed"})
                            elif isinstance(existing_val, float):
                                setattr(self.selected_item, attr_name, float(val_str))
                            elif isinstance(existing_val, int):
                                setattr(self.selected_item, attr_name, int(val_str))
                            else:
                                setattr(self.selected_item, attr_name, val_str)
                        
                        # Trigger UI refresh if layout-altering property changes
                        if prop in ["sensorType", "beltShape"]:
                            self.show_props_for(self.selected_item)
                            
                    except ValueError:
                        pass # Ignore invalid numbers
                        
            # Check if any vertex coordinate input changed
            if hasattr(self.selected_item, "vertexs") and self.selected_item.vertexs:
                for i in range(4):
                    if i < len(self.vertex_x_inputs) and (event.ui_element == self.vertex_x_inputs[i] or event.ui_element == self.vertex_y_inputs[i]):
                        try:
                            x_val = float(self.vertex_x_inputs[i].get_text())
                            y_val = float(self.vertex_y_inputs[i].get_text())
                            self.selected_item.vertexs[i]['x'] = x_val
                            self.selected_item.vertexs[i]['y'] = y_val
                            
                            # Also update the string representation in prop_inputs["beltVertexs"] if it exists
                            if "beltVertexs" in self.prop_inputs:
                                val_str = "; ".join(f"{pt['x']:.2f},{pt['y']:.2f}" for pt in self.selected_item.vertexs)
                                self.prop_inputs["beltVertexs"].set_text(val_str)
                        except ValueError:
                            pass
                        
        return None
    
    def update_props_values(self, item):
        if self.selected_item != item or not item: return
        
        if hasattr(item, "x") and "x" in self.prop_inputs:
            self.prop_inputs["x"].set_text(str(round(item.x, 3)))
        if hasattr(item, "y") and "y" in self.prop_inputs:
            self.prop_inputs["y"].set_text(str(round(item.y, 3)))
            
        if getattr(item, "type", None) == "linear" and getattr(item, "shape", "rectangle") == "quadrilateral" and hasattr(item, "vertexs") and item.vertexs:
            for i in range(4):
                if i < len(item.vertexs):
                    self.vertex_x_inputs[i].set_text(f"{item.vertexs[i]['x']:.3f}")
                    self.vertex_y_inputs[i].set_text(f"{item.vertexs[i]['y']:.3f}")
            if "beltVertexs" in self.prop_inputs:
                val_str = "; ".join(f"{pt['x']:.2f},{pt['y']:.2f}" for pt in item.vertexs)
                self.prop_inputs["beltVertexs"].set_text(val_str)

    def update_sensor_state(self, physics_engine: PhysicsEngine):
        """Call every frame with the physics engine sensor_states dict to refresh display."""
        
        sensor_states = physics_engine.sensor_states
        self._flush_mqtt_logs()
        self._flush_mqtt_decodes()

        if self.manager_clear:
            self.manager_clear = False
            physics_engine.clear_all_parcels()
            
        self.last_sensor_states = dict(sensor_states or {})
        self._refresh_composer_table()
        if isinstance(self.selected_item, ControlButton):
            self.refresh_button_state()
            return
        if not isinstance(self.selected_item, Sensor):
            return
        state = sensor_states.get(self.selected_item.id)
        if state is None:
            return
        if isinstance(state, bool):
            text = f"State: {'TRIGGERED' if state else 'clear'}"
            color = "#ff4444" if state else "#aaaaaa"
        else:
            # laser banner bit string
            text = f"Beams: {state}"
            color = "#44aaff"
        self.sensor_state_lbl.set_text(text)

    def refresh_button_state(self):
        if not isinstance(self.selected_item, ControlButton):
            return
        state = bool(getattr(self.selected_item, "sensor_states", False))
        text = f"Trigger: {'PRESSED' if state else 'released'}"
        self.sensor_state_lbl.set_text(text)
        self.sensor_state_lbl.show()
        inp = self.prop_inputs.get("sensor_states")
        if inp:
            inp.set_text(str(state))

    def open_placement_dialog(self, cmd_type: str):
        if self.placement_window and self.placement_window.alive():
            self.placement_window.kill()
        self.placement_cmd_type = cmd_type
        self.placement_window = pygame_gui.elements.UIWindow(
            rect=pygame.Rect(400, 300, 250, 180),
            manager=self.manager,
            window_display_title="Placement Position",
            resizable=False
        )
        container = self.placement_window.get_container()
        pygame_gui.elements.UILabel(pygame.Rect(10, 10, 30, 28), "X:", self.manager, container=container)
        self.placement_x_input = pygame_gui.elements.UITextEntryLine(pygame.Rect(50, 10, 150, 28), self.manager, container=container)
        self.placement_x_input.set_text("0.0")
        
        pygame_gui.elements.UILabel(pygame.Rect(10, 50, 30, 28), "Y:", self.manager, container=container)
        self.placement_y_input = pygame_gui.elements.UITextEntryLine(pygame.Rect(50, 50, 150, 28), self.manager, container=container)
        self.placement_y_input.set_text("0.0")
        
        self.placement_ok_btn = pygame_gui.elements.UIButton(pygame.Rect(50, 90, 70, 30), "OK", self.manager, container=container)
        self.placement_cancel_btn = pygame_gui.elements.UIButton(pygame.Rect(130, 90, 70, 30), "Cancel", self.manager, container=container)
