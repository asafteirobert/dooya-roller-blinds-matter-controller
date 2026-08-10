/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_err.h>
#include <esp_log.h>
#include <esp_mac.h>
#include <esp_timer.h>
#include <nvs_flash.h>

#include <esp_matter.h>
#include <esp_matter_console.h>
#include <esp_matter_ota.h>

#include <common_macros.h>
#include <log_heap_numbers.h>

#include <app/clusters/window-covering-server/WindowCoveringDelegate.h>
#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>

#include <cstdint>

#include "Constants.hpp"

#include "ButtonDriver.hpp"
#include "BlindController.hpp"
#include "RadioController.hpp"
#include "SX1276Driver.hpp"

#ifdef CONFIG_ENABLE_SET_CERT_DECLARATION_API
#include <esp_matter_providers.h>
#include <lib/support/Span.h>
#ifdef CONFIG_SEC_CERT_DAC_PROVIDER
#include <platform/ESP32/ESP32SecureCertDACProvider.h>
#elif defined(CONFIG_FACTORY_PARTITION_DAC_PROVIDER)
#include <platform/ESP32/ESP32FactoryDataProvider.h>
#endif
using namespace chip::DeviceLayer;
#endif

static const char *TAG = "app_main";

// Maximum number of blinds this firmware can be configured to control. Bounds the NumBlinds
// attribute (see SystemConfig below) and sizes the per-blind static arrays below (this codebase
// does no heap allocation, so the arrays are sized to the hard max and only the first
// numBlinds entries are ever used).
constexpr uint8_t MAX_BLINDS = 8;

ButtonDriver buttonDriver;
RadioController radioController;
SX1276Driver sx1276Driver;

// One BlindController per configured blind. RadioController/SX1276Driver stay singletons: they
// are the shared radio, addressed per-command by remoteId+channel, so only per-blind motion
// state needs multiple instances.
BlindController blindControllers[MAX_BLINDS];
// 0 is the root endpoint id, never a blind's, so it doubles as the "unused slot" sentinel here.
uint16_t windowCoveringEndpointIds[MAX_BLINDS] = {0};
uint8_t numBlinds = 1;

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

constexpr auto TIMEOUT_SECONDS = 300;

// Manufacturer-specific cluster exposing BlindController's per-installation settings (travel
// timings, RF channel, remote ID), so they can be tuned over Matter instead of being
// hardcoded. The standard WindowCovering cluster has no attributes for any of this. Home
// Assistant does not know how to map a manufacturer cluster to an entity, so these are not
// editable from the normal HA UI; they must be read/written through the Matter Server's raw
// attribute interface (e.g. python-matter-server's developer tools).
namespace BlindConfig
{
// CONFIG_DEVICE_VENDOR_ID (0xFFF1) is the Matter test vendor ID this build already uses;
// FC00 is the first cluster ID in the manufacturer-specific range.
constexpr uint32_t CLUSTER_ID = 0xFFF1FC00;
constexpr uint32_t LOWER_TIME_ATTRIBUTE_ID = 0x0000;
constexpr uint32_t RISE_TIME_ATTRIBUTE_ID = 0x0001;
constexpr uint32_t CHANNEL_ATTRIBUTE_ID = 0x0002;
constexpr uint32_t REMOTE_ID_ATTRIBUTE_ID = 0x0003;
constexpr uint32_t MIN_TRAVEL_TIME_MS = 1000;
constexpr uint32_t MAX_TRAVEL_TIME_MS = 120000;
// Dooya remotes commonly address channels 0-15 (0 often used as an "all channels" group).
constexpr uint8_t MIN_CHANNEL = 0;
constexpr uint8_t MAX_CHANNEL = 15;
// The remote ID occupies 3 bytes of the RF packet (see RadioController::sendCommand), so it
// is a 24-bit value.
constexpr uint32_t MAX_REMOTE_ID = 0xFFFFFF;
} // namespace BlindConfig

// Manufacturer-specific cluster on the root endpoint (endpoint 0) exposing how many blind
// endpoints to instantiate. Lives on the root endpoint (rather than a blind's own endpoint)
// because it must be readable before any blind endpoint has been created, since it decides how
// many of those to create. There is no standard Matter cluster for this: endpoint composition
// is normally discovered (via the Descriptor cluster's PartsList), not commanded.
namespace SystemConfig
{
constexpr uint32_t CLUSTER_ID = 0xFFF1FC01;
constexpr uint32_t NUM_BLINDS_ATTRIBUTE_ID = 0x0000;
constexpr uint8_t MIN_BLINDS = 1;
constexpr uint8_t MAX_BLINDS = ::MAX_BLINDS;
} // namespace SystemConfig

// Bridges Matter WindowCovering cluster commands to this instance's BlindController.
class BlindWindowCoveringDelegate : public WindowCovering::WindowCoveringDelegate
{
public:
    void SetBlindController(BlindController *blindController) { this->controller = blindController; }

    CHIP_ERROR HandleMovement(WindowCovering::WindowCoveringType type) override
    {
        attribute_t *target_attribute = attribute::get(this->mEndpoint, WindowCovering::Id,
                                                        WindowCovering::Attributes::TargetPositionLiftPercent100ths::Id);
        esp_matter_attr_val_t target_val = esp_matter_invalid(nullptr);
        attribute::get_val(target_attribute, &target_val);

        this->controller->moveTo(static_cast<uint8_t>(target_val.val.u16 / 100));

        return CHIP_NO_ERROR;
    }

    CHIP_ERROR HandleStopMotion() override
    {
        // Triggers the position-changed callback with the resting position, syncing the Current* attributes.
        this->controller->stop();

        // Per the Matter spec, Stop cancels any pending motion, so Target should collapse to Current.
        // TargetPositionLiftPercent100ths is a nullable attribute, so it must be updated with a
        // nullable value or esp_matter rejects the write with ESP_ERR_INVALID_ARG.
        uint8_t percentage = this->controller->getPositionPercentage();
        esp_matter_attr_val_t position_100ths = esp_matter_nullable_uint16(nullable<uint16_t>(static_cast<uint16_t>(percentage) * 100));
        attribute::update(this->mEndpoint, WindowCovering::Id,
                          WindowCovering::Attributes::TargetPositionLiftPercent100ths::Id, &position_100ths);

        return CHIP_NO_ERROR;
    }

private:
    BlindController *controller = nullptr;
};

static BlindWindowCoveringDelegate windowCoveringDelegates[MAX_BLINDS];

#ifdef CONFIG_ENABLE_SET_CERT_DECLARATION_API
extern const uint8_t cd_start[] asm("_binary_certification_declaration_der_start");
extern const uint8_t cd_end[] asm("_binary_certification_declaration_der_end");

const chip::ByteSpan cdSpan(cd_start, static_cast<size_t>(cd_end - cd_start));
#endif // CONFIG_ENABLE_SET_CERT_DECLARATION_API

#if CONFIG_ENABLE_ENCRYPTED_OTA
extern const char decryption_key_start[] asm("_binary_esp_image_encryption_key_pem_start");
extern const char decryption_key_end[] asm("_binary_esp_image_encryption_key_pem_end");

static const char *DECRYPTION_KEY = decryption_key_start;
static const uint16_t DECRYPTION_KEY_LEN = decryption_key_end - decryption_key_start;
#endif // CONFIG_ENABLE_ENCRYPTED_OTA

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) 
    {
    case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
        ESP_LOGI(TAG, "Interface IP Address changed");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        break;

    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGI(TAG, "Commissioning failed, fail safe timer expired");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
        ESP_LOGI(TAG, "Commissioning session started");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
        ESP_LOGI(TAG, "Commissioning session stopped");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
        ESP_LOGI(TAG, "Commissioning window opened");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
        ESP_LOGI(TAG, "Commissioning window closed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricRemoved: 
    {
        ESP_LOGI(TAG, "Fabric removed successfully");
        if (chip::Server::GetInstance().GetFabricTable().FabricCount() == 0) {
            chip::CommissioningWindowManager  &commissionMgr = chip::Server::GetInstance().GetCommissioningWindowManager();
            constexpr auto COMMISSIONING_WINDOW_TIMEOUT = chip::System::Clock::Seconds16(TIMEOUT_SECONDS);
            if (!commissionMgr.IsCommissioningWindowOpen())
            {
                /* After removing last fabric, this example does not remove the Wi-Fi credentials
                 * and still has IP connectivity so, only advertising on DNS-SD.
                 */
                CHIP_ERROR err = commissionMgr.OpenBasicCommissioningWindow(COMMISSIONING_WINDOW_TIMEOUT,
                                                                            chip::CommissioningWindowAdvertisement::kDnssdOnly);
                if (err != CHIP_NO_ERROR) 
                {
                    ESP_LOGE(TAG, "Failed to open commissioning window, err:%" CHIP_ERROR_FORMAT, err.Format());
                }
            }
        }
        break;
    }

    case chip::DeviceLayer::DeviceEventType::kFabricWillBeRemoved:
        ESP_LOGI(TAG, "Fabric will be removed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricUpdated:
        ESP_LOGI(TAG, "Fabric is updated");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricCommitted:
        ESP_LOGI(TAG, "Fabric is committed");
        break;

    case chip::DeviceLayer::DeviceEventType::kBLEDeinitialized:
        ESP_LOGI(TAG, "BLE deinitialized and memory reclaimed");
        break;

    default:
        break;
    }
}

// This callback is invoked when clients interact with the Identify Cluster.
// In the callback implementation, an endpoint can identify itself. (e.g., by flashing an LED or light).
static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                       uint8_t effect_variant, void *priv_data)
{
    ESP_LOGI(TAG, "Identification callback: type: %u, effect: %u, variant: %u", type, effect_id, effect_variant);
    return ESP_OK;
}

// Finds which blind (if any) an endpoint id belongs to. Linear scan is fine: at most MAX_BLINDS
// (8) entries.
static BlindController *find_blind_controller(uint16_t endpoint_id)
{
    for (uint8_t i = 0; i < numBlinds; ++i)
    {
        if (windowCoveringEndpointIds[i] == endpoint_id)
        {
            return &blindControllers[i];
        }
    }
    return nullptr;
}

static void reboot_timer_callback(void *arg)
{
    esp_restart();
}

// NumBlinds changes which endpoints exist, and this firmware only builds that endpoint set
// once, at boot. Rather than adding/removing endpoints live, apply the change by rebooting
// shortly after the write — the delay gives the write's own acknowledgement time to go out
// over the network before the reset.
static void schedule_reboot()
{
    static esp_timer_handle_t reboot_timer = nullptr;
    if (reboot_timer == nullptr)
    {
        const esp_timer_create_args_t timer_args = {
            .callback = &reboot_timer_callback,
            .arg = nullptr,
            .name = "num_blinds_reboot",
        };
        esp_timer_create(&timer_args, &reboot_timer);
    }
    esp_timer_start_once(reboot_timer, 2000000); // 2s
    ESP_LOGW(TAG, "NumBlinds changed; rebooting in 2s to apply");
}

// This callback is called for every attribute update. The callback implementation shall
// handle the desired attributes and return an appropriate error code. If the attribute
// is not of your interest, please do not return an error code and strictly return ESP_OK.
static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    esp_err_t err = ESP_OK;

    if (type == PRE_UPDATE)
    {
        if (cluster_id == BlindConfig::CLUSTER_ID)
        {
            BlindController *controller = find_blind_controller(endpoint_id);
            if (controller != nullptr)
            {
                if (attribute_id == BlindConfig::LOWER_TIME_ATTRIBUTE_ID)
                {
                    controller->blindLowerTimeMs = val->val.u32;
                }
                else if (attribute_id == BlindConfig::RISE_TIME_ATTRIBUTE_ID)
                {
                    controller->blindRiseTimeMs = val->val.u32;
                }
                else if (attribute_id == BlindConfig::CHANNEL_ATTRIBUTE_ID)
                {
                    controller->blindRadioChannel = val->val.u8;
                }
                else if (attribute_id == BlindConfig::REMOTE_ID_ATTRIBUTE_ID)
                {
                    controller->blindRemoteId = val->val.u32;
                }
            }
        }
    }
    else if (type == POST_UPDATE)
    {
        if (cluster_id == SystemConfig::CLUSTER_ID && attribute_id == SystemConfig::NUM_BLINDS_ATTRIBUTE_ID)
        {
            schedule_reboot();
        }
    }

    return err;
}

extern "C" void app_main()
{
    esp_err_t err = ESP_OK;

    /* Initialize the ESP NVS layer */
    nvs_flash_init();

    /* Create a Matter node and add the mandatory Root Node device type on endpoint 0 */
    node::config_t node_config;

    // node handle can be used to add/modify other endpoints.
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "Failed to create Matter node"));

    // Custom cluster on the root endpoint (always exists, unlike the dynamic blind endpoints
    // created below) exposing how many blind endpoints to instantiate. Must be created and read
    // before the endpoint-creation loop below, since it determines that loop's bound.
    endpoint_t *root_endpoint = endpoint::get(node, 0);
    ABORT_APP_ON_FAILURE(root_endpoint != nullptr, ESP_LOGE(TAG, "Failed to get root endpoint"));

    cluster_t *system_config_cluster = cluster::create(root_endpoint, SystemConfig::CLUSTER_ID, CLUSTER_FLAG_SERVER);
    ABORT_APP_ON_FAILURE(system_config_cluster != nullptr, ESP_LOGE(TAG, "Failed to create system config cluster"));

    attribute_t *num_blinds_attribute = attribute::create(system_config_cluster, SystemConfig::NUM_BLINDS_ATTRIBUTE_ID,
                                                           ATTRIBUTE_FLAG_WRITABLE | ATTRIBUTE_FLAG_NONVOLATILE,
                                                           esp_matter_uint8(1));
    attribute::add_bounds(num_blinds_attribute, esp_matter_uint8(SystemConfig::MIN_BLINDS),
                          esp_matter_uint8(SystemConfig::MAX_BLINDS));

    // This attribute is nonvolatile, so attribute::create() above already restored any value
    // persisted from before a reset (falling back to the default of 1 on first-ever boot). The
    // bounds check above guards normal writes, but a corrupted NVS record (e.g. from a future
    // firmware downgrade) bypasses that path, so clamp defensively before using it as a loop bound.
    esp_matter_attr_val_t restored_num_blinds = esp_matter_invalid(nullptr);
    attribute::get_val(num_blinds_attribute, &restored_num_blinds);
    numBlinds = restored_num_blinds.val.u8;
    if (numBlinds < SystemConfig::MIN_BLINDS || numBlinds > SystemConfig::MAX_BLINDS)
    {
        ESP_LOGW(TAG, "Persisted NumBlinds %u out of range, falling back to 1", numBlinds);
        numBlinds = 1;
    }

    // Initialize the shared radio once; each blind's BlindController below gets its own init()
    // call inside the loop, but they all drive commands through this same radioController.
    buttonDriver.init();
    sx1276Driver.init();
    radioController.init(sx1276Driver);

    for (uint8_t i = 0; i < numBlinds; ++i)
    {
        window_covering::config_t window_covering_config;
        window_covering_config.window_covering.type = (uint8_t)WindowCovering::Type::kRollerShadeExterior;
        window_covering_config.window_covering.feature_flags = cluster::window_covering::feature::lift::get_id() |
                                                                cluster::window_covering::feature::position_aware_lift::get_id();
        // Nullable positions default to null; Home Assistant's discovery skips creating the cover entity
        // entirely if CurrentPositionLiftPercent100ths reads as null, so give it a concrete starting value.
        window_covering_config.window_covering.features.position_aware_lift.current_position_lift_percent_100ths = nullable<uint16_t>(0);
        window_covering_config.window_covering.features.position_aware_lift.target_position_lift_percent_100ths = nullable<uint16_t>(0);
        // Operational + Online. The LiftPositionAware bit is OR'd in automatically when the feature is added below.
        window_covering_config.window_covering.config_status = (uint8_t)WindowCovering::ConfigStatus::kOperational |
                                                                (uint8_t)WindowCovering::ConfigStatus::kOnlineReserved;
        window_covering_config.window_covering.delegate = &windowCoveringDelegates[i];

        // endpoint handles can be used to add/modify clusters.
        endpoint_t *endpoint = window_covering::create(node, &window_covering_config, ENDPOINT_FLAG_NONE, nullptr);
        ABORT_APP_ON_FAILURE(endpoint != nullptr, ESP_LOGE(TAG, "Failed to create window covering endpoint"));

        windowCoveringEndpointIds[i] = endpoint::get_id(endpoint);
        ESP_LOGI(TAG, "Window covering %u created with endpoint_id %d", i, windowCoveringEndpointIds[i]);

        windowCoveringDelegates[i].SetEndpoint(windowCoveringEndpointIds[i]);
        windowCoveringDelegates[i].SetBlindController(&blindControllers[i]);

        // Home Assistant's cover platform keys off this legacy attribute; esp-matter does not add it automatically.
        cluster_t *window_covering_cluster = cluster::get(endpoint, WindowCovering::Id);
        cluster::window_covering::attribute::create_current_position_lift_percentage(window_covering_cluster, nullable<uint8_t>(0));

        /* Mark deferred persistence for the position attribute since it changes rapidly while moving */
        attribute_t *current_position_attribute = attribute::get(windowCoveringEndpointIds[i], WindowCovering::Id,
                                                                  WindowCovering::Attributes::CurrentPositionLiftPercent100ths::Id);
        attribute::set_deferred_persistence(current_position_attribute);

        // This attribute is nonvolatile, so attribute::create() above already restored it from NVS
        // (falling back to the 0 default on first-ever boot). Read it back so BlindController starts
        // from the real last-known position instead of assuming fully open after a reset.
        esp_matter_attr_val_t restored_position_100ths = esp_matter_invalid(nullptr);
        attribute::get_val(current_position_attribute, &restored_position_100ths);
        uint8_t restored_percentage = static_cast<uint8_t>(restored_position_100ths.val.u16 / 100);

        // Custom cluster exposing the blind's travel timings so they can be tuned over Matter.
        cluster_t *blind_config_cluster = cluster::create(endpoint, BlindConfig::CLUSTER_ID, CLUSTER_FLAG_SERVER);
        ABORT_APP_ON_FAILURE(blind_config_cluster != nullptr, ESP_LOGE(TAG, "Failed to create blind config cluster"));

        attribute_t *lower_time_attribute = attribute::create(blind_config_cluster, BlindConfig::LOWER_TIME_ATTRIBUTE_ID,
                                                               ATTRIBUTE_FLAG_WRITABLE | ATTRIBUTE_FLAG_NONVOLATILE,
                                                               esp_matter_uint32(blindControllers[i].blindLowerTimeMs));
        attribute::add_bounds(lower_time_attribute, esp_matter_uint32(BlindConfig::MIN_TRAVEL_TIME_MS),
                              esp_matter_uint32(BlindConfig::MAX_TRAVEL_TIME_MS));

        attribute_t *rise_time_attribute = attribute::create(blind_config_cluster, BlindConfig::RISE_TIME_ATTRIBUTE_ID,
                                                              ATTRIBUTE_FLAG_WRITABLE | ATTRIBUTE_FLAG_NONVOLATILE,
                                                              esp_matter_uint32(blindControllers[i].blindRiseTimeMs));
        attribute::add_bounds(rise_time_attribute, esp_matter_uint32(BlindConfig::MIN_TRAVEL_TIME_MS),
                              esp_matter_uint32(BlindConfig::MAX_TRAVEL_TIME_MS));

        // Fresh-boot default: give each blind a distinct channel (1, 2, 3, ...) instead of every
        // instance defaulting to BlindController's same hardcoded value. A value persisted from
        // before a reset still overrides this via the restored_channel read-back below.
        blindControllers[i].blindRadioChannel = static_cast<uint8_t>(i + 1);
        attribute_t *channel_attribute = attribute::create(blind_config_cluster, BlindConfig::CHANNEL_ATTRIBUTE_ID,
                                                            ATTRIBUTE_FLAG_WRITABLE | ATTRIBUTE_FLAG_NONVOLATILE,
                                                            esp_matter_uint8(blindControllers[i].blindRadioChannel));
        attribute::add_bounds(channel_attribute, esp_matter_uint8(BlindConfig::MIN_CHANNEL),
                              esp_matter_uint8(BlindConfig::MAX_CHANNEL));

        attribute_t *remote_id_attribute = attribute::create(blind_config_cluster, BlindConfig::REMOTE_ID_ATTRIBUTE_ID,
                                                              ATTRIBUTE_FLAG_WRITABLE | ATTRIBUTE_FLAG_NONVOLATILE,
                                                              esp_matter_uint32(blindControllers[i].blindRemoteId));
        attribute::add_bounds(remote_id_attribute, esp_matter_uint32(0), esp_matter_uint32(BlindConfig::MAX_REMOTE_ID));

        // These attributes are nonvolatile, so attribute::create() above already restored any value
        // persisted from before a reset (falling back to the default passed in above on first-ever boot).
        // Read them back so BlindController starts with the real configured values.
        esp_matter_attr_val_t restored_lower_time = esp_matter_invalid(nullptr);
        attribute::get_val(lower_time_attribute, &restored_lower_time);
        blindControllers[i].blindLowerTimeMs = restored_lower_time.val.u32;

        esp_matter_attr_val_t restored_rise_time = esp_matter_invalid(nullptr);
        attribute::get_val(rise_time_attribute, &restored_rise_time);
        blindControllers[i].blindRiseTimeMs = restored_rise_time.val.u32;

        esp_matter_attr_val_t restored_channel = esp_matter_invalid(nullptr);
        attribute::get_val(channel_attribute, &restored_channel);
        blindControllers[i].blindRadioChannel = restored_channel.val.u8;

        esp_matter_attr_val_t restored_remote_id = esp_matter_invalid(nullptr);
        attribute::get_val(remote_id_attribute, &restored_remote_id);
        blindControllers[i].blindRemoteId = restored_remote_id.val.u32;

        // Mirrors this blind's current-position estimate into the Matter data model. Fired on
        // move start/interruption and on natural completion, so Home Assistant sees the resting
        // position even if a move gets redirected partway through. Does not touch the Target
        // attribute: on completion current == target already, and on interruption the Target
        // attribute has just been set to the new target by the caller of HandleMovement, so
        // overwriting it here would clobber that. CurrentPositionLiftPercent100ths and
        // CurrentPositionLiftPercentage were both created as nullable attributes, so they must be
        // updated with nullable values or esp_matter rejects the write with ESP_ERR_INVALID_ARG
        // (silently leaving the attribute, and Home Assistant's display, stale). Captures this
        // blind's endpoint id by value since it never changes after creation.
        uint16_t endpoint_id_for_callback = windowCoveringEndpointIds[i];
        blindControllers[i].init(
            radioController,
            [endpoint_id_for_callback](uint8_t percentage)
            {
                esp_matter_attr_val_t position_100ths =
                    esp_matter_nullable_uint16(nullable<uint16_t>(static_cast<uint16_t>(percentage) * 100));
                attribute::update(endpoint_id_for_callback, WindowCovering::Id,
                                  WindowCovering::Attributes::CurrentPositionLiftPercent100ths::Id, &position_100ths);

                // Kept in sync for clients (e.g. Home Assistant) that only read the legacy percentage attribute.
                esp_matter_attr_val_t position_percentage = esp_matter_nullable_uint8(nullable<uint8_t>(percentage));
                attribute::update(endpoint_id_for_callback, WindowCovering::Id,
                                  WindowCovering::Attributes::CurrentPositionLiftPercentage::Id, &position_percentage);
            },
            restored_percentage);
    }

#ifdef CONFIG_ENABLE_SET_CERT_DECLARATION_API
    auto * dac_provider = get_dac_provider();
#ifdef CONFIG_SEC_CERT_DAC_PROVIDER
    static_cast<ESP32SecureCertDACProvider *>(dac_provider)->SetCertificationDeclaration(cdSpan);
#elif defined(CONFIG_FACTORY_PARTITION_DAC_PROVIDER)
    static_cast<ESP32FactoryDataProvider *>(dac_provider)->SetCertificationDeclaration(cdSpan);
#endif
#endif // CONFIG_ENABLE_SET_CERT_DECLARATION_API

    /* Matter start */
    err = esp_matter::start(app_event_cb);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to start Matter, err:%d", err));

#if CONFIG_ENABLE_ENCRYPTED_OTA
    err = esp_matter_ota_requestor_encrypted_init(DECRYPTION_KEY, DECRYPTION_KEY_LEN);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to initialized the encrypted OTA, err: %d", err));
#endif // CONFIG_ENABLE_ENCRYPTED_OTA

#if CONFIG_ENABLE_CHIP_SHELL
    esp_matter::console::diagnostics_register_commands();
    esp_matter::console::wifi_register_commands();
    esp_matter::console::factoryreset_register_commands();
    esp_matter::console::attribute_register_commands();
#if CONFIG_OPENTHREAD_CLI
    esp_matter::console::otcli_register_commands();
#endif
    esp_matter::console::init();
#endif

    ESP_LOGI(TAG, "Init finished");
    vTaskDelete(nullptr);
}
