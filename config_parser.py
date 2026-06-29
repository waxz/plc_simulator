import json
from typing import List, Optional, Literal, Dict, Any, Union
from pydantic import BaseModel, Field

class Belt(BaseModel):
    id: str
    type: Literal["linear", "curved"]
    shape: Literal["quadrilateral", "arc", "rectangle"] = "rectangle"
    x: float
    y: float
    length: float = 0.0
    beltWidth: float
    rotation: float
    radius: float = 0.0
    startAngle: float = 0.0
    endAngle: float = 0.0
    speed: float
    directionAngle: float
    direction: int = 1
    color: str
    vertexs: Optional[List[Dict[str, float]]] = None
    speedBinding: Optional[str] = None
    directionAngleBinding: Optional[str] = None
    directionBinding: Optional[str] = None
    transition_momentum_duration: float = 0.6
    

class Sensor(BaseModel):
    id: str
    x: float
    y: float
    width: float
    height: float
    rotation: float
    isActive: bool = False
    label: str
    color: str
    stateBinding: Optional[str] = None
    sensorType: Literal["ir", "laser_banner"] = "ir"
    laserCount: Optional[int] = None
    beamDistance: Optional[float] = None
    wordBinding: Optional[str] = None
    detectedWord: Optional[int] = 0
    sensor_states: Optional[Union[bool, int, str]] = None

class Source(BaseModel):
    id: str
    x: float
    y: float
    rotation: float
    interval: float
    minWidth: float
    maxWidth: float
    minHeight: float
    maxHeight: float
    label: str
    colorScheme: Optional[List[str]] = None
    triggerBinding: Optional[str] = None
    intervalBinding: Optional[str] = None

class Sink(BaseModel):
    id: str
    x: float
    y: float
    width: float
    height: float
    label: str

class ControlButton(BaseModel):
    id: str
    x: float
    y: float
    width: float = 0.6
    height: float = 0.4
    rotation: float = 0.0
    label: str
    color: str = "#3b82f6"
    sensor_states: Optional[Union[bool, int, str]] = None
    stateBinding: Optional[str] = None

class LedIndicator(BaseModel):
    id: str
    x: float
    y: float
    radius: float = 0.2
    label: str
    color: str = "#22c55e"
    isOn: bool = False
    stateBinding: Optional[str] = None

class TopicComponent(BaseModel):
    id: str
    type: Literal["belt", "sensor", "item", "laser_banner", "button", "led"]

class TopicConfig(BaseModel):
    id: str
    topic: str
    interval: int
    direction: Literal["in", "out"]
    format: Literal["json", "raw", "string", "binary_struct"]
    endianness: Literal["LE", "BE"]
    components: List[TopicComponent] = Field(default_factory=list)

class MqttSettings(BaseModel):
    enabled: bool = False
    transport: Literal["mqtt", "tcp"] = "mqtt"
    brokerUrl: str = "ws://localhost"
    port: int = 5000
    topicPrefix: str = ""
    tcpBindHost: str = "0.0.0.0"
    tcpServerHost: str = "127.0.0.1"
    tcpPort: int = 5001
    endianness: Literal["LE", "BE"] = "LE"
    topics: List[TopicConfig] = Field(default_factory=list)

class SimulatorConfig(BaseModel):
    version: str = "1.0.0"
    savedAt: int = 0
    belts: List[Belt] = Field(default_factory=list)
    sensors: List[Sensor] = Field(default_factory=list)
    sources: List[Source] = Field(default_factory=list)
    sinks: List[Sink] = Field(default_factory=list)
    buttons: List[ControlButton] = Field(default_factory=list)
    leds: List[LedIndicator] = Field(default_factory=list)
    mqttSettings: MqttSettings = Field(default_factory=MqttSettings)
    simulatorBackend: Literal["matter", "rapier"] = "matter"
    simulationSteps: int = 4
    kineticFriction: bool = False
    collisionEnabled: bool = True
    gridSnap: bool = True
    gridSize: float = 0.5
    useSurfaceVelocity: bool = False

def load_config(file_path: str) -> SimulatorConfig:
    with open(file_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    return SimulatorConfig(**data)

if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        config = load_config(sys.argv[1])
        print(f"Loaded config with {len(config.belts)} belts, {len(config.sensors)} sensors.")
