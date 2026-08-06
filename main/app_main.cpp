/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_err.h>
#include <esp_log.h>
#include <esp_mac.h>
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
uint16_t window_covering_endpoint_id = 0;

ButtonDriver buttonDriver;
BlindController blindController; //TODO multiple instances when required
RadioController radioController;
SX1276Driver sx1276Driver;

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;

constexpr auto k_timeout_seconds = 300;

// Bridges Matter WindowCovering cluster commands to BlindController.
class BlindWindowCoveringDelegate : public WindowCovering::WindowCoveringDelegate
{
public:
    CHIP_ERROR HandleMovement(WindowCovering::WindowCoveringType type) override
    {
        attribute_t *target_attribute = attribute::get(window_covering_endpoint_id, WindowCovering::Id,
                                                        WindowCovering::Attributes::TargetPositionLiftPercent100ths::Id);
        esp_matter_attr_val_t target_val = esp_matter_invalid(nullptr);
        attribute::get_val(target_attribute, &target_val);

        blindController.moveTo(static_cast<uint8_t>(target_val.val.u16 / 100));

        return CHIP_NO_ERROR;
    }

    CHIP_ERROR HandleStopMotion() override
    {
        blindController.stop();

        uint8_t percentage = blindController.getPositionPercentage();
        esp_matter_attr_val_t position_100ths = esp_matter_uint16(static_cast<uint16_t>(percentage) * 100);
        attribute::update(window_covering_endpoint_id, WindowCovering::Id,
                          WindowCovering::Attributes::CurrentPositionLiftPercent100ths::Id, &position_100ths);
        attribute::update(window_covering_endpoint_id, WindowCovering::Id,
                          WindowCovering::Attributes::TargetPositionLiftPercent100ths::Id, &position_100ths);

        // Kept in sync for clients (e.g. Home Assistant) that only read the legacy percentage attribute.
        esp_matter_attr_val_t position_percentage = esp_matter_uint8(percentage);
        attribute::update(window_covering_endpoint_id, WindowCovering::Id,
                          WindowCovering::Attributes::CurrentPositionLiftPercentage::Id, &position_percentage);

        return CHIP_NO_ERROR;
    }
};

static BlindWindowCoveringDelegate windowCoveringDelegate;

#ifdef CONFIG_ENABLE_SET_CERT_DECLARATION_API
extern const uint8_t cd_start[] asm("_binary_certification_declaration_der_start");
extern const uint8_t cd_end[] asm("_binary_certification_declaration_der_end");

const chip::ByteSpan cdSpan(cd_start, static_cast<size_t>(cd_end - cd_start));
#endif // CONFIG_ENABLE_SET_CERT_DECLARATION_API

#if CONFIG_ENABLE_ENCRYPTED_OTA
extern const char decryption_key_start[] asm("_binary_esp_image_encryption_key_pem_start");
extern const char decryption_key_end[] asm("_binary_esp_image_encryption_key_pem_end");

static const char *s_decryption_key = decryption_key_start;
static const uint16_t s_decryption_key_len = decryption_key_end - decryption_key_start;
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
            constexpr auto kTimeoutSeconds = chip::System::Clock::Seconds16(k_timeout_seconds);
            if (!commissionMgr.IsCommissioningWindowOpen()) 
            {
                /* After removing last fabric, this example does not remove the Wi-Fi credentials
                 * and still has IP connectivity so, only advertising on DNS-SD.
                 */
                CHIP_ERROR err = commissionMgr.OpenBasicCommissioningWindow(kTimeoutSeconds,
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

// This callback is called for every attribute update. The callback implementation shall
// handle the desired attributes and return an appropriate error code. If the attribute
// is not of your interest, please do not return an error code and strictly return ESP_OK.
static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    esp_err_t err = ESP_OK;

    if (type == PRE_UPDATE) 
    {
        /* Driver update */
        // TODO
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
    window_covering_config.window_covering.delegate = &windowCoveringDelegate;

    // endpoint handles can be used to add/modify clusters.
    endpoint_t *endpoint = window_covering::create(node, &window_covering_config, ENDPOINT_FLAG_NONE, nullptr);
    ABORT_APP_ON_FAILURE(endpoint != nullptr, ESP_LOGE(TAG, "Failed to create window covering endpoint"));

    window_covering_endpoint_id = endpoint::get_id(endpoint);
    ESP_LOGI(TAG, "Window covering created with endpoint_id %d", window_covering_endpoint_id);

    windowCoveringDelegate.SetEndpoint(window_covering_endpoint_id);

    // Home Assistant's cover platform keys off this legacy attribute; esp-matter does not add it automatically.
    cluster_t *window_covering_cluster = cluster::get(endpoint, WindowCovering::Id);
    cluster::window_covering::attribute::create_current_position_lift_percentage(window_covering_cluster, nullable<uint8_t>(0));

    /* Mark deferred persistence for the position attribute since it changes rapidly while moving */
    attribute_t *current_position_attribute = attribute::get(window_covering_endpoint_id, WindowCovering::Id,
                                                              WindowCovering::Attributes::CurrentPositionLiftPercent100ths::Id);
    attribute::set_deferred_persistence(current_position_attribute);

    // Initialize drivers
    buttonDriver.init();
    sx1276Driver.init();
    radioController.init(sx1276Driver);
    blindController.init(radioController, 1);

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
    err = esp_matter_ota_requestor_encrypted_init(s_decryption_key, s_decryption_key_len);
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
