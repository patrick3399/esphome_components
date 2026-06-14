from esphome import automation
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import light, number, select, sensor, switch
from esphome.components.socket import SocketType, consume_sockets
from esphome.const import (
    CONF_BLUE,
    CONF_BRIGHTNESS,
    CONF_GREEN,
    CONF_ID,
    CONF_RED,
    CONF_WHITE,
    Framework,
    PLATFORM_ESP32,
)
from esphome.core import CORE
from pathlib import Path

try:
    from esphome.components.esp32 import add_idf_sdkconfig_option
    _HAS_SDKCONFIG = True
except ImportError:
    _HAS_SDKCONFIG = False

DEPENDENCIES = ["light"]
AUTO_LOAD = ["web_server_base", "json", "select", "number", "switch", "sensor"]

wled_bridge_ns = cg.esphome_ns.namespace("wled_bridge")
WLEDBridgeComponent = wled_bridge_ns.class_("WLEDBridgeComponent", cg.Component)

# Automation action classes
LoadPresetAction = wled_bridge_ns.class_("LoadPresetAction", automation.Action)
SavePresetAction = wled_bridge_ns.class_("SavePresetAction", automation.Action)
SetEffectAction = wled_bridge_ns.class_("SetEffectAction", automation.Action)
SetPaletteAction = wled_bridge_ns.class_("SetPaletteAction", automation.Action)
SetBrightnessAction = wled_bridge_ns.class_("SetBrightnessAction", automation.Action)
SetColorAction = wled_bridge_ns.class_("SetColorAction", automation.Action)
PowerAction = wled_bridge_ns.class_("PowerAction", automation.Action)
StopPlaylistAction = wled_bridge_ns.class_("StopPlaylistAction", automation.Action)
SetSpeedAction = wled_bridge_ns.class_("SetSpeedAction", automation.Action)
SetIntensityAction = wled_bridge_ns.class_("SetIntensityAction", automation.Action)
StartNightlightAction = wled_bridge_ns.class_("StartNightlightAction", automation.Action)
StopNightlightAction = wled_bridge_ns.class_("StopNightlightAction", automation.Action)
DeletePresetAction = wled_bridge_ns.class_("DeletePresetAction", automation.Action)
SetSegmentBoundsAction = wled_bridge_ns.class_("SetSegmentBoundsAction", automation.Action)
SetTransitionAction = wled_bridge_ns.class_("SetTransitionAction", automation.Action)
SetSegmentReverseAction = wled_bridge_ns.class_("SetSegmentReverseAction", automation.Action)
SetSegmentMirrorAction = wled_bridge_ns.class_("SetSegmentMirrorAction", automation.Action)

# HA entity classes (select / number)
WLEDPaletteSelect = wled_bridge_ns.class_(
    "WLEDPaletteSelect", select.Select, cg.Component
)
WLEDEffectSelect = wled_bridge_ns.class_(
    "WLEDEffectSelect", select.Select, cg.Component
)
WLEDNumber = wled_bridge_ns.class_("WLEDNumber", number.Number, cg.Component)
WLED_NUM_SPEED = wled_bridge_ns.enum("WLEDNumberProperty").WLED_NUM_SPEED
WLED_NUM_INTENSITY = wled_bridge_ns.enum("WLEDNumberProperty").WLED_NUM_INTENSITY

# New HA entity classes
WLEDPresetSelect = wled_bridge_ns.class_(
    "WLEDPresetSelect", select.Select, cg.Component
)
WLEDNightlightSwitch = wled_bridge_ns.class_(
    "WLEDNightlightSwitch", switch.Switch, cg.Component
)
WLEDSyncSendSwitch = wled_bridge_ns.class_(
    "WLEDSyncSendSwitch", switch.Switch, cg.Component
)
WLEDSyncReceiveSwitch = wled_bridge_ns.class_(
    "WLEDSyncReceiveSwitch", switch.Switch, cg.Component
)
WLEDCurrentSensor = wled_bridge_ns.class_(
    "WLEDCurrentSensor", sensor.Sensor, cg.Component
)

CONF_LIGHT_ID = "light_id"
CONF_BUSES = "buses"
CONF_TYPE = "type"
CONF_MAX_MA = "max_ma"
CONF_LED_MA = "led_ma"
CONF_USE_TASK = "use_task"
CONF_AUTO_WHITE = "auto_white"
CONF_MATRIX_WIDTH = "matrix_width"
CONF_MATRIX_HEIGHT = "matrix_height"
CONF_MATRIX_SERPENTINE = "matrix_serpentine"
CONF_UDP_PORT = "udp_port"
CONF_UDP_PORT2 = "udp_port2"
CONF_UDP_SEND = "udp_send"
CONF_UDP_RECEIVE = "udp_receive"
CONF_BOOT_PRESET = "boot_preset"
CONF_BRIGHTNESS_FACTOR = "brightness_factor"
CONF_WEB_UI = "web_ui"
CONF_REALTIME = "realtime"
CONF_DDP = "ddp"
CONF_E131 = "e131"
CONF_E131_UNIVERSE = "e131_universe"
CONF_E131_UNIVERSES = "e131_universes"
CONF_ARTNET = "artnet"
CONF_ARTNET_UNIVERSE = "artnet_universe"
CONF_ARTNET_UNIVERSES = "artnet_universes"

# Kept for backward compatibility — flat keys still accepted alongside nested realtime:
CONF_DDP_RECEIVE = "ddp_receive"
CONF_E131_RECEIVE = "e131_receive"
CONF_E131_UNIVERSE_COUNT = "e131_universe_count"
CONF_ARTNET_RECEIVE = "artnet_receive"
CONF_ARTNET_UNIVERSE_COUNT = "artnet_universe_count"

# HA entity config keys
CONF_ENTITIES = "entities"
CONF_PALETTE_SELECT = "palette_select"
CONF_EFFECT_SELECT = "effect_select"
CONF_SPEED_NUMBER = "speed_number"
CONF_INTENSITY_NUMBER = "intensity_number"
CONF_PRESET_SELECT = "preset_select"
CONF_NIGHTLIGHT_SWITCH = "nightlight_switch"
CONF_SYNC_SEND_SWITCH = "sync_send_switch"
CONF_SYNC_RECEIVE_SWITCH = "sync_receive_switch"
CONF_ESTIMATED_CURRENT = "estimated_current"

CONF_AUDIO = "audio"
CONF_AUDIO_MICROPHONE = "microphone"
CONF_AUDIO_PASSIVE = "passive"
CONF_AUDIO_FFT = "fft"
CONF_AUDIO_AGC = "agc"

# WLED's sound-reactive engine (volume + FFT/GEQ binning) assumes this fixed
# sample rate; it must match AUDIO_SAMPLE_RATE in wled_audio.h. The microphone
# must be capable of providing it, or the FFT frequency bins are miscalibrated.
AUDIO_SAMPLE_RATE = 22050

# Auto-white modes for RGBW strips (derive W channel from RGB).
AUTO_WHITE_MODES = {
    "none": 0,
    "brighter": 1,
    "accurate": 2,
    "max": 4,
}

BUS_TYPES = {
    "addressable": "addressable",
    "mono": "mono",
    "monochromatic": "mono",
    "on_off": "on_off",
    "rgb": "rgb",
    "rgbw": "rgbw",
    "rgbww": "rgbww",
    "rgbct": "rgbct",
    "cct": "cct",
    "cwww": "cct",
    "cold_warm_white": "cct",
}

BUS_CAPABILITIES = {
    "mono": 0x02,
    "on_off": 0x00,
    "rgb": 0x01,
    "rgbw": 0x03,
    "rgbww": 0x07,
    "rgbct": 0x07,
    "cct": 0x06,
}

DDP_PORT = 4048
E131_PORT = 5568
ARTNET_PORT = 6454
E131_MAX_UNIVERSE = 63999
ARTNET_MAX_UNIVERSE = 32767
WLED_MODE_MAX = 219
WLED_PALETTE_MAX = 71

ADDRESSABLE_BUS_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_TYPE, default="addressable"): cv.one_of(
            "addressable", lower=True
        ),
        cv.Required(CONF_LIGHT_ID): cv.use_id(light.AddressableLightState),
        cv.Optional(CONF_MAX_MA, default=5000): cv.int_range(min=0),
        cv.Optional(CONF_LED_MA, default=55): cv.positive_int,
    }
)

LIGHTSTATE_BUS_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_TYPE): cv.enum(BUS_TYPES, lower=True),
        cv.Required(CONF_LIGHT_ID): cv.use_id(light.LightState),
        cv.Optional(CONF_MAX_MA, default=5000): cv.int_range(min=0),
        cv.Optional(CONF_LED_MA, default=55): cv.positive_int,
    }
)


def _validate_bus(value):
    bus_type = str(value.get(CONF_TYPE, "addressable")).lower()
    if BUS_TYPES.get(bus_type, bus_type) == "addressable":
        return ADDRESSABLE_BUS_SCHEMA(value)
    return LIGHTSTATE_BUS_SCHEMA(value)


BUS_SCHEMA = cv.Schema(_validate_bus)

# Nested realtime: section schema
REALTIME_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_DDP, default=False): cv.boolean,
        cv.Optional(CONF_E131, default=False): cv.boolean,
        cv.Optional(CONF_E131_UNIVERSE, default=1): cv.int_range(min=1, max=63999),
        cv.Optional(CONF_E131_UNIVERSES, default=1): cv.int_range(min=1, max=8),
        cv.Optional(CONF_ARTNET, default=False): cv.boolean,
        cv.Optional(CONF_ARTNET_UNIVERSE, default=0): cv.int_range(min=0, max=32767),
        cv.Optional(CONF_ARTNET_UNIVERSES, default=1): cv.int_range(min=1, max=8),
    }
)


# Nested audio: section — enables sound reactive effects via ESPHome microphone.
# Uses a validator function to defer `microphone` import until config validation,
# because wled_bridge does not hard-depend on the microphone component.
def _validate_audio(value):
    from esphome.components import microphone  # noqa: F811
    schema = cv.Schema(
        {
            cv.Optional(CONF_AUDIO_MICROPHONE, default={}):
                microphone.microphone_source_schema(
                    min_bits_per_sample=16, max_bits_per_sample=16,
                ),
            cv.Optional(CONF_AUDIO_PASSIVE, default=False): cv.boolean,
            cv.Optional(CONF_AUDIO_FFT, default=False): cv.boolean,
            cv.Optional(CONF_AUDIO_AGC, default=True): cv.boolean,
        }
    )
    return schema(value)


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(WLEDBridgeComponent),
        # Accept either a single light_id (backward compat) or a buses: list.
        cv.Optional(CONF_LIGHT_ID): cv.use_id(light.AddressableLightState),
        cv.Optional(CONF_BUSES): cv.All(cv.ensure_list(BUS_SCHEMA), cv.Length(min=1)),
        # Global defaults used when light_id (not buses:) is specified.
        cv.Optional(CONF_MAX_MA): cv.int_range(min=0),
        cv.Optional(CONF_LED_MA): cv.positive_int,
        cv.Optional(CONF_USE_TASK, default=False): cv.boolean,
        cv.Optional(CONF_AUTO_WHITE, default="none"): cv.enum(
            AUTO_WHITE_MODES, lower=True
        ),
        # 2D matrix geometry — required for 2D effects
        cv.Optional(CONF_MATRIX_WIDTH, default=0): cv.int_range(min=0, max=256),
        cv.Optional(CONF_MATRIX_HEIGHT, default=0): cv.int_range(min=0, max=256),
        cv.Optional(CONF_MATRIX_SERPENTINE, default=False): cv.boolean,
        # UDP WLED notifier sync
        cv.Optional(CONF_UDP_PORT, default=21324): cv.port,
        cv.Optional(CONF_UDP_PORT2, default=65506): cv.port,
        cv.Optional(CONF_UDP_SEND, default=False): cv.boolean,
        cv.Optional(CONF_UDP_RECEIVE, default=False): cv.boolean,
        # Boot preset (0 = restore last NVS state, 1-16 = load specific preset)
        cv.Optional(CONF_BOOT_PRESET, default=0): cv.int_range(min=0, max=16),
        # Brightness factor (0-100%, caps maximum brightness like WLED BF setting)
        cv.Optional(CONF_BRIGHTNESS_FACTOR, default=100): cv.int_range(min=0, max=100),
        # Web UI (default enabled): set false to strip ~64 KB of gzip blobs from flash
        cv.Optional(CONF_WEB_UI, default=True): cv.boolean,
        # Nested realtime: section — enables DDP/E1.31/Art-Net receivers
        cv.Optional(CONF_REALTIME): REALTIME_SCHEMA,
        # Nested audio: section — enables sound reactive effects via ESPHome microphone
        cv.Optional(CONF_AUDIO): _validate_audio,
        # HA entities — auto-create select/number entities for HA dashboard control.
        # Each key accepts a select/number schema dict for naming, or false to disable.
        cv.Optional(CONF_PALETTE_SELECT): select.select_schema(WLEDPaletteSelect),
        cv.Optional(CONF_EFFECT_SELECT): select.select_schema(WLEDEffectSelect),
        cv.Optional(CONF_SPEED_NUMBER): number.number_schema(
            WLEDNumber,
            icon="mdi:speedometer",
        ),
        cv.Optional(CONF_INTENSITY_NUMBER): number.number_schema(
            WLEDNumber,
            icon="mdi:brightness-percent",
        ),
        cv.Optional(CONF_PRESET_SELECT): select.select_schema(
            WLEDPresetSelect,
            icon="mdi:bookmark",
        ),
        cv.Optional(CONF_NIGHTLIGHT_SWITCH): switch.switch_schema(
            WLEDNightlightSwitch,
            icon="mdi:weather-night",
        ),
        cv.Optional(CONF_SYNC_SEND_SWITCH): switch.switch_schema(
            WLEDSyncSendSwitch,
            icon="mdi:upload-network",
        ),
        cv.Optional(CONF_SYNC_RECEIVE_SWITCH): switch.switch_schema(
            WLEDSyncReceiveSwitch,
            icon="mdi:download-network",
        ),
        cv.Optional(CONF_ESTIMATED_CURRENT): sensor.sensor_schema(
            WLEDCurrentSensor,
            unit_of_measurement="mA",
            accuracy_decimals=0,
            device_class="current",
            state_class="measurement",
            icon="mdi:flash",
        ),
        # --- Backward-compat flat keys (deprecated, use realtime: section instead) ---
        cv.Optional(CONF_DDP_RECEIVE, default=False): cv.boolean,
        cv.Optional(CONF_E131_RECEIVE, default=False): cv.boolean,
        cv.Optional(CONF_E131_UNIVERSE, default=1): cv.int_range(min=1, max=63999),
        cv.Optional(CONF_E131_UNIVERSE_COUNT, default=1): cv.int_range(min=1, max=8),
        cv.Optional(CONF_ARTNET_RECEIVE, default=False): cv.boolean,
        cv.Optional(CONF_ARTNET_UNIVERSE, default=0): cv.int_range(min=0, max=32767),
        cv.Optional(CONF_ARTNET_UNIVERSE_COUNT, default=1): cv.int_range(min=1, max=8),
    }
).extend(cv.COMPONENT_SCHEMA)


def _validate(config):
    if CONF_LIGHT_ID not in config and CONF_BUSES not in config:
        raise cv.Invalid("Either 'light_id' or 'buses' must be specified")
    if CONF_LIGHT_ID in config and CONF_BUSES in config:
        raise cv.Invalid("'light_id' and 'buses' are mutually exclusive")
    if config[CONF_USE_TASK]:
        raise cv.Invalid("'use_task' is disabled until the render path is thread-safe")
    matrix_width = config[CONF_MATRIX_WIDTH]
    matrix_height = config[CONF_MATRIX_HEIGHT]
    if (matrix_width == 0) != (matrix_height == 0):
        raise cv.Invalid("'matrix_width' and 'matrix_height' must both be 0 or both be greater than 0")
    if matrix_width > 0 and matrix_height > 0 and matrix_width * matrix_height > 65535:
        raise cv.Invalid(
            "'matrix_width' * 'matrix_height' must not exceed 65535 LEDs "
            "because WLED segment coordinates are 16-bit"
        )
    rt = config.get(CONF_REALTIME)
    if rt is not None:
        flat_overrides = []
        if config.get(CONF_DDP_RECEIVE, False):
            flat_overrides.append(CONF_DDP_RECEIVE)
        if config.get(CONF_E131_RECEIVE, False):
            flat_overrides.append(CONF_E131_RECEIVE)
        if config.get(CONF_E131_UNIVERSE, 1) != 1:
            flat_overrides.append(CONF_E131_UNIVERSE)
        if config.get(CONF_E131_UNIVERSE_COUNT, 1) != 1:
            flat_overrides.append(CONF_E131_UNIVERSE_COUNT)
        if config.get(CONF_ARTNET_RECEIVE, False):
            flat_overrides.append(CONF_ARTNET_RECEIVE)
        if config.get(CONF_ARTNET_UNIVERSE, 0) != 0:
            flat_overrides.append(CONF_ARTNET_UNIVERSE)
        if config.get(CONF_ARTNET_UNIVERSE_COUNT, 1) != 1:
            flat_overrides.append(CONF_ARTNET_UNIVERSE_COUNT)
        if flat_overrides:
            raise cv.Invalid(
                "'realtime' cannot be mixed with deprecated flat realtime keys: "
                + ", ".join(flat_overrides)
            )
        ddp_enabled = rt.get(CONF_DDP, False)
        e131_enabled = rt.get(CONF_E131, False)
        e131_universe = rt.get(CONF_E131_UNIVERSE, 1)
        e131_count = rt.get(CONF_E131_UNIVERSES, 1)
        artnet_enabled = rt.get(CONF_ARTNET, False)
        artnet_universe = rt.get(CONF_ARTNET_UNIVERSE, 0)
        artnet_count = rt.get(CONF_ARTNET_UNIVERSES, 1)
    else:
        ddp_enabled = config.get(CONF_DDP_RECEIVE, False)
        e131_enabled = config.get(CONF_E131_RECEIVE, False)
        e131_universe = config.get(CONF_E131_UNIVERSE, 1)
        e131_count = config.get(CONF_E131_UNIVERSE_COUNT, 1)
        artnet_enabled = config.get(CONF_ARTNET_RECEIVE, False)
        artnet_universe = config.get(CONF_ARTNET_UNIVERSE, 0)
        artnet_count = config.get(CONF_ARTNET_UNIVERSE_COUNT, 1)
    if e131_enabled and e131_universe + e131_count - 1 > E131_MAX_UNIVERSE:
        raise cv.Invalid("E1.31 universe range exceeds 63999")
    if artnet_enabled and artnet_universe + artnet_count - 1 > ARTNET_MAX_UNIVERSE:
        raise cv.Invalid("Art-Net universe range exceeds 32767")
    if config.get(CONF_UDP_RECEIVE, False):
        udp_ports = {config[CONF_UDP_PORT], config[CONF_UDP_PORT2]}
        realtime_ports = set()
        if ddp_enabled:
            realtime_ports.add(DDP_PORT)
        if e131_enabled:
            realtime_ports.add(E131_PORT)
        if artnet_enabled:
            realtime_ports.add(ARTNET_PORT)
        conflict = udp_ports & realtime_ports
        if conflict:
            raise cv.Invalid(f"UDP sync receive port conflicts with realtime receiver port {min(conflict)}")
    return config


def _has_realtime(config):
    """Return True when any realtime receiver is enabled (nested or flat keys)."""
    rt = config.get(CONF_REALTIME)
    if rt is not None:
        if rt.get(CONF_DDP, False) or rt.get(CONF_E131, False) or rt.get(CONF_ARTNET, False):
            return True
    if config.get(CONF_DDP_RECEIVE, False):
        return True
    if config.get(CONF_E131_RECEIVE, False):
        return True
    if config.get(CONF_ARTNET_RECEIVE, False):
        return True
    return False


def _count_udp_sockets(config):
    count = 0
    if config.get(CONF_UDP_SEND, False):
        count += 1  # send broadcast socket
    if config.get(CONF_UDP_RECEIVE, False):
        count += 1  # recv port1
        if config.get(CONF_UDP_PORT2, 0) != config.get(CONF_UDP_PORT, 21324):
            count += 1  # recv port2
    rt = config.get(CONF_REALTIME) or {}
    if rt.get(CONF_DDP, False) or config.get(CONF_DDP_RECEIVE, False):
        count += 1  # DDP port 4048
    if rt.get(CONF_E131, False) or config.get(CONF_E131_RECEIVE, False):
        count += 1  # E1.31/sACN port 5568
    if rt.get(CONF_ARTNET, False) or config.get(CONF_ARTNET_RECEIVE, False):
        count += 1  # Art-Net port 6454
    return count


def _consume_wled_bridge_sockets(config):
    return consume_sockets(
        _count_udp_sockets(config),
        "wled_bridge",
        SocketType.UDP,
    )(config)


CONFIG_SCHEMA = cv.All(
    CONFIG_SCHEMA,
    cv.only_on(PLATFORM_ESP32),
    cv.only_with_framework(Framework.ESP_IDF),
    _validate,
    _consume_wled_bridge_sockets,
)


def _final_validate(config):
    """Ensure the configured microphone can provide WLED's expected sample rate.

    The C++ audio analyzer / FFT (wled_audio.h, wled_audio_fft.cpp) assume a
    fixed AUDIO_SAMPLE_RATE; without this check a mismatched mic rate would
    silently miscalibrate the GEQ bands and fft_major_peak frequency.
    """
    audio = config.get(CONF_AUDIO)
    if audio is None:
        return config
    from esphome.components import microphone  # noqa: F811

    microphone.final_validate_microphone_source_schema(
        "wled_bridge audio", sample_rate=AUDIO_SAMPLE_RATE
    )(audio[CONF_AUDIO_MICROPHONE])
    return config


FINAL_VALIDATE_SCHEMA = _final_validate

# Path to pre-compressed UI asset blobs generated by tools/gen_wled_ui.py
_UI_ASSETS_DIR = Path(__file__).parent / "ui_assets"

# Maps symbol prefix → (gz filename, size symbol suffix)
_UI_BLOBS = [
    ("WLED_INDEX", "index.htm.gz"),
    ("WLED_INDEX_JS", "index.js.gz"),
    ("WLED_INDEX_CSS", "index.css.gz"),
    ("WLED_RANGETOUCH", "rangetouch.js.gz"),
    ("WLED_IRO", "iro.js.gz"),
    ("WLED_SETTINGS", "settings.html.gz"),
]


def _inject_ui_blobs():
    """Inject PROGMEM blob globals from pre-compressed .gz files.

    Each blob becomes two global declarations in the generated C++ file:
      const uint8_t SYMBOL_GZ[] PROGMEM = { 0xNN, ... };
      const size_t SYMBOL_GZ_SIZE = NNN;

    Using PROGMEM (= placed in flash) keeps the blobs out of RAM.
    """
    for symbol, gz_filename in _UI_BLOBS:
        gz_path = _UI_ASSETS_DIR / gz_filename
        if not gz_path.exists():
            raise FileNotFoundError(
                f"UI asset not found: {gz_path}. "
                "Run tools/gen_wled_ui.py to regenerate."
            )
        gz_data = gz_path.read_bytes()
        hex_bytes = ", ".join(f"0x{b:02x}" for b in gz_data)
        array_decl = (
            f"namespace esphome {{ namespace wled_bridge {{ "
            f"const uint8_t {symbol}_GZ[] PROGMEM = {{ {hex_bytes} }}; "
            f"const size_t {symbol}_GZ_SIZE = {len(gz_data)}u; "
            f"}} }}"
        )
        cg.add_global(cg.RawExpression(array_decl))


def FILTER_SOURCE_FILES():
    """Exclude source files for disabled subsystems.

    Called by ESPHome with no arguments; reads the component config from CORE.

    - wled_udp.cpp: always excluded (replaced by wled_udp_sync.cpp + wled_udp_realtime.cpp)
    - wled_effects_2d.cpp: excluded when no 2D matrix is configured
    - wled_udp_realtime.cpp: excluded when realtime: section is absent / all disabled
    """
    domain = "wled_bridge"
    raw = CORE.config.get(domain)
    # ESPHome stores single-instance component config either as a dict directly,
    # or as a list of dicts when MULTI_CONF is set.
    if isinstance(raw, list):
        config = raw[0] if raw else {}
    elif isinstance(raw, dict):
        config = raw
    else:
        config = {}
    # wled_udp.cpp: always excluded — split into wled_udp_sync.cpp + wled_udp_realtime.cpp
    # wled_ui_data.cpp: always excluded — replaced by Python-side PROGMEM injection in to_code()
    filtered = ["wled_udp.cpp", "wled_ui_data.cpp"]
    if config.get(CONF_MATRIX_WIDTH, 0) == 0 or config.get(CONF_MATRIX_HEIGHT, 0) == 0:
        filtered.append("wled_effects_2d.cpp")
    if not _has_realtime(config):
        filtered.append("wled_udp_realtime.cpp")
    # HA entities: exclude when no select/number entities are configured
    has_ent = any(
        k in config
        for k in (CONF_PALETTE_SELECT, CONF_EFFECT_SELECT,
                  CONF_SPEED_NUMBER, CONF_INTENSITY_NUMBER,
                  CONF_PRESET_SELECT, CONF_NIGHTLIGHT_SWITCH,
                  CONF_SYNC_SEND_SWITCH, CONF_SYNC_RECEIVE_SWITCH,
                  CONF_ESTIMATED_CURRENT)
    )
    if not has_ent:
        filtered.append("wled_entities.cpp")
    # Audio: three files, two gates
    audio = config.get(CONF_AUDIO)
    if audio is None:
        filtered.append("wled_audio.cpp")
        filtered.append("wled_audio_fft.cpp")
        filtered.append("wled_effects_audio.cpp")
    elif not audio.get(CONF_AUDIO_FFT, False):
        filtered.append("wled_audio_fft.cpp")
    return filtered


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_use_task(config[CONF_USE_TASK]))
    cg.add(var.set_auto_white_mode(config[CONF_AUTO_WHITE]))

    # 2D matrix
    cg.add(var.set_matrix_width(config[CONF_MATRIX_WIDTH]))
    cg.add(var.set_matrix_height(config[CONF_MATRIX_HEIGHT]))
    cg.add(var.set_matrix_serpentine(config[CONF_MATRIX_SERPENTINE]))
    if config[CONF_MATRIX_WIDTH] > 0 and config[CONF_MATRIX_HEIGHT] > 0:
        cg.add_define("WLED_BRIDGE_2D")

    # UDP sync
    cg.add(var.set_udp_port(config[CONF_UDP_PORT]))
    cg.add(var.set_udp_port2(config[CONF_UDP_PORT2]))
    cg.add(var.set_udp_send(config[CONF_UDP_SEND]))
    cg.add(var.set_udp_receive(config[CONF_UDP_RECEIVE]))

    # Boot preset
    if config[CONF_BOOT_PRESET] > 0:
        cg.add(var.set_boot_preset(config[CONF_BOOT_PRESET]))

    # Brightness factor
    if config[CONF_BRIGHTNESS_FACTOR] < 100:
        cg.add(var.set_brightness_factor(config[CONF_BRIGHTNESS_FACTOR]))

    # Web UI blobs (Layer B + D)
    if config[CONF_WEB_UI]:
        cg.add_define("WLED_BRIDGE_WEB_UI")
        _inject_ui_blobs()

    # Realtime receivers — nested realtime: section (Layer C)
    rt = config.get(CONF_REALTIME)
    if rt is not None:
        ddp_en = rt.get(CONF_DDP, False)
        e131_en = rt.get(CONF_E131, False)
        artnet_en = rt.get(CONF_ARTNET, False)
        if ddp_en or e131_en or artnet_en:
            cg.add_define("WLED_BRIDGE_REALTIME")
        cg.add(var.set_ddp_enabled(ddp_en))
        cg.add(var.set_e131_enabled(e131_en))
        cg.add(var.set_e131_universe(rt.get(CONF_E131_UNIVERSE, 1)))
        cg.add(var.set_e131_universe_count(rt.get(CONF_E131_UNIVERSES, 1)))
        cg.add(var.set_artnet_enabled(artnet_en))
        cg.add(var.set_artnet_universe(rt.get(CONF_ARTNET_UNIVERSE, 0)))
        cg.add(var.set_artnet_universe_count(rt.get(CONF_ARTNET_UNIVERSES, 1)))
    else:
        # Backward-compat flat keys
        ddp_en = config.get(CONF_DDP_RECEIVE, False)
        e131_en = config.get(CONF_E131_RECEIVE, False)
        artnet_en = config.get(CONF_ARTNET_RECEIVE, False)
        if ddp_en or e131_en or artnet_en:
            cg.add_define("WLED_BRIDGE_REALTIME")
        cg.add(var.set_ddp_enabled(ddp_en))
        cg.add(var.set_e131_enabled(e131_en))
        cg.add(var.set_e131_universe(config[CONF_E131_UNIVERSE]))
        cg.add(var.set_e131_universe_count(config[CONF_E131_UNIVERSE_COUNT]))
        cg.add(var.set_artnet_enabled(artnet_en))
        cg.add(var.set_artnet_universe(config[CONF_ARTNET_UNIVERSE]))
        cg.add(var.set_artnet_universe_count(config[CONF_ARTNET_UNIVERSE_COUNT]))

    # Enable IDF WebSocket support so /ws endpoint works
    if _HAS_SDKCONFIG and CORE.is_esp32:
        add_idf_sdkconfig_option("CONFIG_HTTPD_WS_SUPPORT", True)

    # Audio reactive — ESPHome MicrophoneSource integration
    audio = config.get(CONF_AUDIO)
    if audio is not None:
        from esphome.components import microphone  # noqa: F811
        cg.add_define("WLED_BRIDGE_AUDIO")
        mic_source = await microphone.microphone_source_to_code(
            audio[CONF_AUDIO_MICROPHONE], passive=audio[CONF_AUDIO_PASSIVE]
        )
        cg.add(var.set_audio_source(mic_source))
        cg.add(var.set_audio_agc(audio[CONF_AUDIO_AGC]))
        if audio[CONF_AUDIO_FFT]:
            cg.add_define("WLED_BRIDGE_FFT")
            cg.add(var.set_audio_fft(True))

    if CONF_BUSES in config:
        # Multi-bus path: call add_bus() for each entry in order.
        for bus_cfg in config[CONF_BUSES]:
            light_state = await cg.get_variable(bus_cfg[CONF_LIGHT_ID])
            bus_type = bus_cfg.get(CONF_TYPE, "addressable")
            if bus_type == "addressable":
                cg.add(var.add_bus(light_state, bus_cfg[CONF_MAX_MA], bus_cfg[CONF_LED_MA]))
            else:
                cg.add(
                    var.add_light_bus(
                        light_state,
                        bus_cfg[CONF_MAX_MA],
                        bus_cfg[CONF_LED_MA],
                        BUS_CAPABILITIES.get(bus_type, 0),
                    )
                )
    else:
        # Single light_id backward-compat path: one bus, global max_ma/led_ma.
        light_state = await cg.get_variable(config[CONF_LIGHT_ID])
        cg.add(
            var.add_bus(
                light_state, config.get(CONF_MAX_MA, 5000), config.get(CONF_LED_MA, 55)
            )
        )

    # ---- HA entities: select / number / switch / sensor ----
    has_entities = any(
        k in config
        for k in (CONF_PALETTE_SELECT, CONF_EFFECT_SELECT,
                  CONF_SPEED_NUMBER, CONF_INTENSITY_NUMBER,
                  CONF_PRESET_SELECT, CONF_NIGHTLIGHT_SWITCH,
                  CONF_SYNC_SEND_SWITCH, CONF_SYNC_RECEIVE_SWITCH,
                  CONF_ESTIMATED_CURRENT)
    )
    if has_entities:
        cg.add_define("WLED_BRIDGE_ENTITIES")

    # Note: options=["_"] is a placeholder — the real option list is populated
    # by the C++ setup() from the compiled WLED_PALETTES / WLED_EFFECTS tables.
    if CONF_PALETTE_SELECT in config:
        sel = await select.new_select(
            config[CONF_PALETTE_SELECT], options=["_"]
        )
        await cg.register_component(sel, config[CONF_PALETTE_SELECT])
        cg.add(sel.set_bridge(var))

    if CONF_EFFECT_SELECT in config:
        sel = await select.new_select(
            config[CONF_EFFECT_SELECT], options=["_"]
        )
        await cg.register_component(sel, config[CONF_EFFECT_SELECT])
        cg.add(sel.set_bridge(var))

    if CONF_SPEED_NUMBER in config:
        num = await number.new_number(
            config[CONF_SPEED_NUMBER],
            min_value=0,
            max_value=255,
            step=1,
        )
        await cg.register_component(num, config[CONF_SPEED_NUMBER])
        cg.add(num.set_bridge(var))
        cg.add(num.set_property(WLED_NUM_SPEED))

    if CONF_INTENSITY_NUMBER in config:
        num = await number.new_number(
            config[CONF_INTENSITY_NUMBER],
            min_value=0,
            max_value=255,
            step=1,
        )
        await cg.register_component(num, config[CONF_INTENSITY_NUMBER])
        cg.add(num.set_bridge(var))
        cg.add(num.set_property(WLED_NUM_INTENSITY))

    if CONF_PRESET_SELECT in config:
        sel = await select.new_select(
            config[CONF_PRESET_SELECT], options=["_"]
        )
        await cg.register_component(sel, config[CONF_PRESET_SELECT])
        cg.add(sel.set_bridge(var))

    if CONF_NIGHTLIGHT_SWITCH in config:
        sw = await switch.new_switch(config[CONF_NIGHTLIGHT_SWITCH])
        await cg.register_component(sw, config[CONF_NIGHTLIGHT_SWITCH])
        cg.add(sw.set_bridge(var))

    if CONF_SYNC_SEND_SWITCH in config:
        sw = await switch.new_switch(config[CONF_SYNC_SEND_SWITCH])
        await cg.register_component(sw, config[CONF_SYNC_SEND_SWITCH])
        cg.add(sw.set_bridge(var))

    if CONF_SYNC_RECEIVE_SWITCH in config:
        sw = await switch.new_switch(config[CONF_SYNC_RECEIVE_SWITCH])
        await cg.register_component(sw, config[CONF_SYNC_RECEIVE_SWITCH])
        cg.add(sw.set_bridge(var))

    if CONF_ESTIMATED_CURRENT in config:
        sens = await sensor.new_sensor(config[CONF_ESTIMATED_CURRENT])
        await cg.register_component(sens, config[CONF_ESTIMATED_CURRENT])
        cg.add(sens.set_bridge(var))


CONF_PRESET = "preset"
CONF_EFFECT = "effect"
CONF_PALETTE = "palette"
CONF_SPEED = "speed"
CONF_INTENSITY = "intensity"
CONF_DURATION = "duration"
CONF_TARGET_BRIGHTNESS = "target_brightness"
CONF_MODE = "mode"
CONF_START = "start"
CONF_STOP = "stop"
CONF_TRANSITION = "transition"
CONF_REVERSE = "reverse"
CONF_MIRROR = "mirror"


@automation.register_action(
    "wled_bridge.load_preset",
    LoadPresetAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(WLEDBridgeComponent),
            cv.Required(CONF_PRESET): cv.templatable(cv.int_range(min=1, max=16)),
        }
    ),
    synchronous=True,
)
async def wled_bridge_load_preset_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    template_ = await cg.templatable(config[CONF_PRESET], args, cg.uint8)
    cg.add(var.set_preset(template_))
    return var


@automation.register_action(
    "wled_bridge.save_preset",
    SavePresetAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(WLEDBridgeComponent),
            cv.Required(CONF_PRESET): cv.templatable(cv.int_range(min=1, max=16)),
        }
    ),
    synchronous=True,
)
async def wled_bridge_save_preset_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    template_ = await cg.templatable(config[CONF_PRESET], args, cg.uint8)
    cg.add(var.set_preset(template_))
    return var


@automation.register_action(
    "wled_bridge.set_effect",
    SetEffectAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(WLEDBridgeComponent),
            cv.Required(CONF_EFFECT): cv.templatable(cv.int_range(min=0, max=WLED_MODE_MAX)),
        }
    ),
    synchronous=True,
)
async def wled_bridge_set_effect_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    template_ = await cg.templatable(config[CONF_EFFECT], args, cg.uint8)
    cg.add(var.set_effect(template_))
    return var


@automation.register_action(
    "wled_bridge.set_palette",
    SetPaletteAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(WLEDBridgeComponent),
            cv.Required(CONF_PALETTE): cv.templatable(cv.int_range(min=0, max=WLED_PALETTE_MAX)),
        }
    ),
    synchronous=True,
)
async def wled_bridge_set_palette_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    template_ = await cg.templatable(config[CONF_PALETTE], args, cg.uint8)
    cg.add(var.set_palette(template_))
    return var


@automation.register_action(
    "wled_bridge.set_brightness",
    SetBrightnessAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(WLEDBridgeComponent),
            cv.Required(CONF_BRIGHTNESS): cv.templatable(
                cv.int_range(min=0, max=255)
            ),
        }
    ),
    synchronous=True,
)
async def wled_bridge_set_brightness_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    template_ = await cg.templatable(config[CONF_BRIGHTNESS], args, cg.uint8)
    cg.add(var.set_brightness(template_))
    return var


@automation.register_action(
    "wled_bridge.set_color",
    SetColorAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(WLEDBridgeComponent),
            cv.Required(CONF_RED): cv.templatable(cv.int_range(min=0, max=255)),
            cv.Required(CONF_GREEN): cv.templatable(cv.int_range(min=0, max=255)),
            cv.Required(CONF_BLUE): cv.templatable(cv.int_range(min=0, max=255)),
            cv.Optional(CONF_WHITE, default=0): cv.templatable(
                cv.int_range(min=0, max=255)
            ),
        }
    ),
    synchronous=True,
)
async def wled_bridge_set_color_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_red(await cg.templatable(config[CONF_RED], args, cg.uint8)))
    cg.add(var.set_green(await cg.templatable(config[CONF_GREEN], args, cg.uint8)))
    cg.add(var.set_blue(await cg.templatable(config[CONF_BLUE], args, cg.uint8)))
    cg.add(var.set_white(await cg.templatable(config[CONF_WHITE], args, cg.uint8)))
    return var


@automation.register_action(
    "wled_bridge.power_on",
    PowerAction,
    cv.Schema({cv.GenerateID(): cv.use_id(WLEDBridgeComponent)}),
    synchronous=True,
)
async def wled_bridge_power_on_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_state(True))
    return var


@automation.register_action(
    "wled_bridge.power_off",
    PowerAction,
    cv.Schema({cv.GenerateID(): cv.use_id(WLEDBridgeComponent)}),
    synchronous=True,
)
async def wled_bridge_power_off_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_state(False))
    return var


@automation.register_action(
    "wled_bridge.power_toggle",
    PowerAction,
    cv.Schema({cv.GenerateID(): cv.use_id(WLEDBridgeComponent)}),
    synchronous=True,
)
async def wled_bridge_power_toggle_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_toggle(True))
    return var


@automation.register_action(
    "wled_bridge.stop_playlist",
    StopPlaylistAction,
    cv.Schema({cv.GenerateID(): cv.use_id(WLEDBridgeComponent)}),
    synchronous=True,
)
async def wled_bridge_stop_playlist_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var


@automation.register_action(
    "wled_bridge.set_speed",
    SetSpeedAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(WLEDBridgeComponent),
            cv.Required(CONF_SPEED): cv.templatable(cv.int_range(min=0, max=255)),
        }
    ),
    synchronous=True,
)
async def wled_bridge_set_speed_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    template_ = await cg.templatable(config[CONF_SPEED], args, cg.uint8)
    cg.add(var.set_speed(template_))
    return var


@automation.register_action(
    "wled_bridge.set_intensity",
    SetIntensityAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(WLEDBridgeComponent),
            cv.Required(CONF_INTENSITY): cv.templatable(
                cv.int_range(min=0, max=255)
            ),
        }
    ),
    synchronous=True,
)
async def wled_bridge_set_intensity_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    template_ = await cg.templatable(config[CONF_INTENSITY], args, cg.uint8)
    cg.add(var.set_intensity(template_))
    return var


@automation.register_action(
    "wled_bridge.start_nightlight",
    StartNightlightAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(WLEDBridgeComponent),
            cv.Optional(CONF_DURATION, default=60): cv.templatable(
                cv.int_range(min=1, max=65535)
            ),
            cv.Optional(CONF_TARGET_BRIGHTNESS, default=0): cv.templatable(
                cv.int_range(min=0, max=255)
            ),
            cv.Optional(CONF_MODE, default=1): cv.templatable(
                cv.int_range(min=0, max=3)
            ),
        }
    ),
    synchronous=True,
)
async def wled_bridge_start_nightlight_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_duration(
        await cg.templatable(config[CONF_DURATION], args, cg.uint16)
    ))
    cg.add(var.set_target_brightness(
        await cg.templatable(config[CONF_TARGET_BRIGHTNESS], args, cg.uint8)
    ))
    cg.add(var.set_mode(
        await cg.templatable(config[CONF_MODE], args, cg.uint8)
    ))
    return var


@automation.register_action(
    "wled_bridge.stop_nightlight",
    StopNightlightAction,
    cv.Schema({cv.GenerateID(): cv.use_id(WLEDBridgeComponent)}),
    synchronous=True,
)
async def wled_bridge_stop_nightlight_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var


@automation.register_action(
    "wled_bridge.delete_preset",
    DeletePresetAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(WLEDBridgeComponent),
            cv.Required(CONF_PRESET): cv.templatable(cv.int_range(min=1, max=16)),
        }
    ),
    synchronous=True,
)
async def wled_bridge_delete_preset_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    template_ = await cg.templatable(config[CONF_PRESET], args, cg.uint8)
    cg.add(var.set_preset(template_))
    return var


@automation.register_action(
    "wled_bridge.set_segment_bounds",
    SetSegmentBoundsAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(WLEDBridgeComponent),
            cv.Required(CONF_START): cv.templatable(cv.positive_int),
            cv.Required(CONF_STOP): cv.templatable(cv.positive_int),
        }
    ),
    synchronous=True,
)
async def wled_bridge_set_segment_bounds_to_code(
    config, action_id, template_arg, args
):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    cg.add(var.set_start(
        await cg.templatable(config[CONF_START], args, cg.uint32)
    ))
    cg.add(var.set_stop(
        await cg.templatable(config[CONF_STOP], args, cg.uint32)
    ))
    return var


@automation.register_action(
    "wled_bridge.set_transition",
    SetTransitionAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(WLEDBridgeComponent),
            cv.Required(CONF_TRANSITION): cv.templatable(
                cv.int_range(min=0, max=65535)
            ),
        }
    ),
    synchronous=True,
)
async def wled_bridge_set_transition_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    template_ = await cg.templatable(config[CONF_TRANSITION], args, cg.uint16)
    cg.add(var.set_transition(template_))
    return var


@automation.register_action(
    "wled_bridge.set_segment_reverse",
    SetSegmentReverseAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(WLEDBridgeComponent),
            cv.Required(CONF_REVERSE): cv.templatable(cv.boolean),
        }
    ),
    synchronous=True,
)
async def wled_bridge_set_segment_reverse_to_code(
    config, action_id, template_arg, args
):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    template_ = await cg.templatable(config[CONF_REVERSE], args, cg.bool_)
    cg.add(var.set_reverse(template_))
    return var


@automation.register_action(
    "wled_bridge.set_segment_mirror",
    SetSegmentMirrorAction,
    cv.Schema(
        {
            cv.GenerateID(): cv.use_id(WLEDBridgeComponent),
            cv.Required(CONF_MIRROR): cv.templatable(cv.boolean),
        }
    ),
    synchronous=True,
)
async def wled_bridge_set_segment_mirror_to_code(
    config, action_id, template_arg, args
):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    template_ = await cg.templatable(config[CONF_MIRROR], args, cg.bool_)
    cg.add(var.set_mirror(template_))
    return var
