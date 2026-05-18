import logging

from esphome import automation
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.automation import register_action, register_condition
from esphome.components import esp32, microphone, ota
from esphome.const import CONF_ID, CONF_INTERNAL, CONF_MICROPHONE, CONF_MODEL

_LOGGER = logging.getLogger(__name__)

CODEOWNERS = ["@patrick3399"]
DEPENDENCIES = ["microphone", "esp32"]

CONF_MODELS = "models"
CONF_ON_WAKE_WORD_DETECTED = "on_wake_word_detected"
CONF_STOP_AFTER_DETECTION = "stop_after_detection"
CONF_WAKE_WORD = "wake_word"
CONF_WAKE_WORDS = "wake_words"


WAKE_WORD_MODELS = {
    "wn9_alexa",
    "wn9_astrolabe_tts",
    "wn9_bluechip_tts2",
    "wn9_computer_tts",
    "wn9_hai1xiao3ou1_tts3",
    "wn9_hai1xiao3xiang4_tts3",
    "wn9_haixiaowu_tts",
    "wn9_heyily_tts2",
    "wn9_heyivy_tts2",
    "wn9_heykira_tts3",
    "wn9_heyprinter_tts",
    "wn9_heywanda_tts",
    "wn9_heywillow_tts",
    "wn9_hiandy_tts2",
    "wn9_hiesp",
    "wn9_hifairy_tts2",
    "wn9_hijason_tts2",
    "wn9_hijolly_tts2",
    "wn9_hijoy_tts",
    "wn9_hilexin",
    "wn9_hilili_tts",
    "wn9_himfive",
    "wn9_himiaomiao_tts",
    "wn9_hitelly_tts",
    "wn9_hiwalle_tts2",
    "wn9_jarvis_tts",
    "wn9_linaiban_tts2",
    "wn9_miaomiaotongxue_tts",
    "wn9_mycroft_tts",
    "wn9_ni3hao3xiao3mai4_tts2",
    "wn9_ni3hao3xiao3rui4_tts3",
    "wn9_nihaobaiying_tts2",
    "wn9_nihaodongdong_tts2",
    "wn9_nihaomiaoban_tts2",
    "wn9_nihaoxiaoan_tts2",
    "wn9_nihaoxiaoxin_tts",
    "wn9_nihaoxiaoyi_tts2",
    "wn9_nihaoxiaozhi_tts",
    "wn9_sophia_tts",
    "wn9_xiao3feng1xiao3feng1_tts3",
    "wn9_xiao3jia1xiao3jia1_tts3",
    "wn9_xiaoaitongxue",
    "wn9_xiaobinxiaobin_tts",
    "wn9_xiaojianxiaojian_tts2",
    "wn9_xiaokangtongxue_tts2",
    "wn9_xiaolongxiaolong_tts",
    "wn9_xiaoluxiaolu_tts2",
    "wn9_xiaomeitongxue_tts",
    "wn9_xiaomingtongxue_tts2",
    "wn9_xiaosurou_tts2",
    "wn9_xiaotexiaote_tts2",
    "wn9_xiaoyaxiaoya_tts2",
    "wn9_xiaoyutongxue_tts2",
    "wn9l_fr_bonjouresp_tts3",
    "wn9l_heygigi_tts3",
    "wn9l_histackchan_tts3",
    "wn9l_ja_konnichihaesp_tts3",
    "wn9l_ni3hao3xing1bao3_tts3",
    "wn9l_nihaoxiaozhi_tts3",
    "wn9l_xiaoaitongxue",
    "wn9s_hiesp",
    "wn9s_hijason",
    "wn9s_hilexin",
    "wn9s_nihaoxiaozhi",
}


esp_sr_ns = cg.esphome_ns.namespace("esp_sr")
EspSrWakeWord = esp_sr_ns.class_("EspSrWakeWord", cg.Component)
EspSrWakeWordModel = esp_sr_ns.class_("EspSrWakeWordModel")

DisableModelAction = esp_sr_ns.class_("DisableModelAction", automation.Action)
EnableModelAction = esp_sr_ns.class_("EnableModelAction", automation.Action)
StartAction = esp_sr_ns.class_("StartAction", automation.Action)
StopAction = esp_sr_ns.class_("StopAction", automation.Action)

ModelIsEnabledCondition = esp_sr_ns.class_(
    "ModelIsEnabledCondition", automation.Condition
)
IsRunningCondition = esp_sr_ns.class_("IsRunningCondition", automation.Condition)


def _validate_models_unique(value):
    model_names = [model[CONF_MODEL] for model in value]
    if len(model_names) != len(set(model_names)):
        raise cv.Invalid("duplicate wake word models are not allowed")
    return value


MODEL_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(EspSrWakeWordModel),
        cv.Required(CONF_MODEL): cv.one_of(*sorted(WAKE_WORD_MODELS), lower=True),
        cv.Optional(CONF_WAKE_WORD): cv.string,
        cv.Optional(CONF_INTERNAL, default=False): cv.boolean,
    }
)


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(EspSrWakeWord),
            cv.Optional(CONF_MICROPHONE, default={}): microphone.microphone_source_schema(
                min_bits_per_sample=16,
                max_bits_per_sample=16,
                min_channels=1,
                max_channels=1,
            ),
            cv.Required(CONF_MODELS): cv.All(
                cv.ensure_list(cv.maybe_simple_value(MODEL_SCHEMA, key=CONF_MODEL)),
                cv.Length(min=1, max=5),
                _validate_models_unique,
            ),
            cv.Optional(CONF_ON_WAKE_WORD_DETECTED): automation.validate_automation(
                single=True
            ),
            cv.Optional(CONF_STOP_AFTER_DETECTION, default=True): cv.boolean,
            cv.Optional(CONF_WAKE_WORDS): cv.invalid(
                f"The {CONF_WAKE_WORDS} option has been replaced by {CONF_MODELS}."
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on_esp32,
    cv.only_with_framework("esp-idf"),
)


FINAL_VALIDATE_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_MICROPHONE): microphone.final_validate_microphone_source_schema(
            "esp_sr_wake_word", sample_rate=16000
        ),
    },
    extra=cv.ALLOW_EXTRA,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    mic_source = await microphone.microphone_source_to_code(config[CONF_MICROPHONE])
    cg.add(var.set_microphone_source(mic_source))

    cg.add_define("USE_ESP_SR_WAKE_WORD")
    ota.request_ota_state_listeners()

    esp32.add_idf_component(name="espressif/esp-sr", ref="2.4.4")

    for i, model_config in enumerate(config[CONF_MODELS]):
        model_dir = model_config[CONF_MODEL]
        flag_suffix = model_dir.upper()
        wake_word = model_config.get(CONF_WAKE_WORD, model_dir)
        default_enabled = i == 0

        model = cg.new_Pvariable(
            model_config[CONF_ID],
            str(model_config[CONF_ID]),
            model_dir,
            wake_word,
            default_enabled,
            model_config[CONF_INTERNAL],
        )
        cg.add(var.add_wake_word_model(model))
        esp32.add_idf_sdkconfig_option(f"CONFIG_SR_WN_{flag_suffix}", True)
        _LOGGER.info(
            "esp_sr_wake_word: enabling wake word '%s' (model: %s)",
            wake_word,
            model_dir,
        )

    cg.add(var.set_stop_after_detection(config[CONF_STOP_AFTER_DETECTION]))

    if on_detection_config := config.get(CONF_ON_WAKE_WORD_DETECTED):
        await automation.build_automation(
            var.get_wake_word_detected_trigger(),
            [(cg.std_string, "wake_word")],
            on_detection_config,
        )


ESP_SR_ACTION_SCHEMA = cv.Schema({cv.GenerateID(): cv.use_id(EspSrWakeWord)})


@register_action(
    "esp_sr_wake_word.start", StartAction, ESP_SR_ACTION_SCHEMA, synchronous=True
)
@register_action(
    "esp_sr_wake_word.stop", StopAction, ESP_SR_ACTION_SCHEMA, synchronous=True
)
@register_condition(
    "esp_sr_wake_word.is_running", IsRunningCondition, ESP_SR_ACTION_SCHEMA
)
async def esp_sr_action_to_code(config, action_id, template_arg, args):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var


ESP_SR_MODEL_ACTION_SCHEMA = automation.maybe_simple_id(
    {
        cv.Required(CONF_ID): cv.use_id(EspSrWakeWordModel),
    }
)


@register_action(
    "esp_sr_wake_word.enable_model",
    EnableModelAction,
    ESP_SR_MODEL_ACTION_SCHEMA,
    synchronous=True,
)
@register_action(
    "esp_sr_wake_word.disable_model",
    DisableModelAction,
    ESP_SR_MODEL_ACTION_SCHEMA,
    synchronous=True,
)
@register_condition(
    "esp_sr_wake_word.model_is_enabled",
    ModelIsEnabledCondition,
    ESP_SR_MODEL_ACTION_SCHEMA,
)
async def model_action_to_code(config, action_id, template_arg, args):
    parent = await cg.get_variable(config[CONF_ID])
    return cg.new_Pvariable(action_id, template_arg, parent)
