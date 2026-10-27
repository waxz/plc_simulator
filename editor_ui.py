import pygame
import pygame_gui
from copy import deepcopy
from config_parser import Belt, Sensor, Source, Sink, ControlButton, LedIndicator
from physics import PhysicsEngine
from id_utils import unique_incremented_id


TOP_BAR = 30     # px, simulator status bar
BOTTOM_BAR = 24  # px, key hint bar


class EditorUI:
    def __init__(self, manager: pygame_gui.UIManager, width: int, height: int, config=None):
        self.manager = manager
        self.width = width
        self.height = height
        self.config = config
        
        # Save Button
        self.save_btn = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(10, TOP_BAR + 8, 120, 28),
            text='Save Layout',
            manager=self.manager
        )
        
        # Toolbar (Left)
        self.toolbar_panel = pygame_gui.elements.UIPanel(
            relative_rect=pygame.Rect(10, TOP_BAR + 44, 120, 330),
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

        # Properties Panel (Right)
        self.prop_panel = pygame_gui.elements.UIPanel(
            relative_rect=pygame.Rect(self.width - 250, TOP_BAR, 250, self._panel_height()),
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
            relative_rect=pygame.Rect(5, self._panel_height() - 85, 235, 28),
            text="State: --",
            manager=self.manager,
            container=self.prop_panel
        )
        self.sensor_state_lbl.hide()
        self.prop_error_lbl = pygame_gui.elements.UILabel(
            relative_rect=pygame.Rect(5, self._panel_height() - 65, 235, 20),
            text="",
            manager=self.manager,
            container=self.prop_panel
        )
        self.prop_error_lbl.hide()
        self.prop_delete_btn = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(5, self._panel_height() - 45, 115, 30),
            text="Delete",
            manager=self.manager,
            container=self.prop_panel
        )
        self.prop_copy_btn = pygame_gui.elements.UIButton(
            relative_rect=pygame.Rect(125, self._panel_height() - 45, 115, 30),
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
        self._clear_prop_error()

        for lbl in getattr(self, 'vertex_labels', []):
            lbl.hide()
        for ix in getattr(self, 'vertex_x_inputs', []):
            ix.hide()
        for iy in getattr(self, 'vertex_y_inputs', []):
            iy.hide()

    def _show_prop_error(self, message: str):
        lbl = getattr(self, 'prop_error_lbl', None)
        if not lbl:
            return
        lbl.set_text(message)
        lbl.show()

    def _clear_prop_error(self):
        lbl = getattr(self, 'prop_error_lbl', None)
        if lbl:
            lbl.set_text("")
            lbl.hide()

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
            elif item.type == "path":
                props_to_show += ["beltWidth"]  # anchors are edited in layout_editor.py
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
        for window in [self.manager_window]:
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
        return unique_incremented_id(existing, base_id)

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
                return new_item
        return None

    def _panel_height(self) -> int:
        return max(200, self.height - TOP_BAR - BOTTOM_BAR)

    def handle_resize(self, width: int, height: int):
        self.width = width
        self.height = height
        self.prop_panel.set_position((width - 250, TOP_BAR))
        self.prop_panel.set_dimensions((250, self._panel_height()))
        # Update delete/copy buttons to anchor to bottom
        if getattr(self, 'prop_delete_btn', None):
            self.prop_delete_btn.set_relative_position((5, self._panel_height() - 45))
        if getattr(self, 'prop_copy_btn', None):
            self.prop_copy_btn.set_relative_position((125, self._panel_height() - 45))
        if getattr(self, 'prop_error_lbl', None):
            self.prop_error_lbl.set_relative_position((5, self._panel_height() - 65))
        if getattr(self, 'sensor_state_lbl', None):
            self.sensor_state_lbl.set_relative_position((5, self._panel_height() - 85))
        # Keep any floating windows from being stranded off-screen
        if self.manager_window and self.manager_window.alive():
            self._clamp_window_to_screen(self.manager_window)
        if self.placement_window and self.placement_window.alive():
            self._clamp_window_to_screen(self.placement_window)

    def _clamp_window_to_screen(self, window: pygame_gui.elements.UIWindow):
        rect = window.rect.copy()
        rect.x = max(0, min(rect.x, self.width - rect.width))
        rect.y = max(0, min(rect.y, self.height - rect.height))
        window.set_position((rect.x, rect.y))

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

        if event.type == pygame_gui.UI_SELECTION_LIST_NEW_SELECTION:
            if event.ui_element == self.manager_list:
                self.manager_selected_key = event.text
                self._update_manager_details()

        if event.type == pygame_gui.UI_WINDOW_CLOSE:
            if event.ui_element == self.manager_window:
                self.manager_window = None

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
                        self._clear_prop_error()

                    except ValueError:
                        # Don't revert the field here: the user may still be
                        # mid-edit (e.g. typing "-" before "-5"). Just surface
                        # that the current text isn't a valid value yet; the
                        # model keeps its last valid value until this parses.
                        self._show_prop_error(f"Invalid value for '{prop}' - not applied")
                        
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

        if self.manager_clear:
            self.manager_clear = False
            physics_engine.clear_all_parcels()

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
