/*
 * BLERemoteDescriptor.cpp
 *
 *  Created on: Jul 8, 2017
 *      Author: kolban
 */
#include "sdkconfig.h"
#if defined(CONFIG_BT_ENABLED)
#include <sstream>
#include "BLERemoteDescriptor.h"
#include "GeneralUtils.h"
#include <esp_log.h>
#ifdef ARDUINO_ARCH_ESP32
#include "esp32-hal-log.h"
#endif

static const char* LOG_TAG = "BLERemoteDescriptor";


BLERemoteDescriptor::BLERemoteDescriptor(
	uint16_t                 handle,
	BLEUUID                  uuid,
	BLERemoteCharacteristic* pRemoteCharacteristic) {

	m_handle                = handle;
	m_uuid                  = uuid;
	m_pRemoteCharacteristic = pRemoteCharacteristic;
}


/**
 * @brief Retrieve the handle associated with this remote descriptor.
 * @return The handle associated with this remote descriptor.
 */
uint16_t BLERemoteDescriptor::getHandle() {
	return m_handle;
} // getHandle


/**
 * @brief Get the characteristic that owns this descriptor.
 * @return The characteristic that owns this descriptor.
 */
BLERemoteCharacteristic* BLERemoteDescriptor::getRemoteCharacteristic() {
	return m_pRemoteCharacteristic;
} // getRemoteCharacteristic


/**
 * @brief Retrieve the UUID associated this remote descriptor.
 * @return The UUID associated this remote descriptor.
 */
BLEUUID BLERemoteDescriptor::getUUID() {
	return m_uuid;
} // getUUID

void BLERemoteDescriptor::gattClientEventHandler(
	esp_gattc_cb_event_t      event,
	esp_gatt_if_t             gattc_if,
	esp_ble_gattc_cb_param_t* evtParam) {
	switch(event) {

		//
		// ESP_GATTC_READ_DESCR_EVT
		// This event indicates that the server has responded to the read request.
		//
		// read:
		// - esp_gatt_status_t  status
		// - uint16_t           conn_id
		// - uint16_t           handle
		// - uint8_t*           value
		// - uint16_t           value_len
	  //
		case ESP_GATTC_READ_DESCR_EVT: {
			// If this event is not for us, then nothing further to do.
			if (evtParam->read.handle != getHandle()) {
				break;
			}

			// At this point, we have determined that the event is for us, so now we save the value
			// and unlock the semaphore to ensure that the requestor of the data can continue.
			if (evtParam->read.status == ESP_GATT_OK) {
				m_value = std::string((char*)evtParam->read.value, evtParam->read.value_len);
			} else {
				m_value = "";
			}

			m_semaphoreReadDescrEvt.give();
			break;
		} // ESP_GATTC_READ_DESCR_EVT
		//
		// ESP_GATTC_WRITE_DESCR_EVT
		//
		// write:
		// - esp_gatt_status_t status
		// - uint16_t          conn_id
		// - uint16_t          handle
		//
		case ESP_GATTC_WRITE_DESCR_EVT: {
			// Determine if this event is for us and, if not, pass onwards.
			if (evtParam->write.handle != getHandle()) {
				break;
			}

			// There is nothing further we need to do here.  
			// This is merely an indication that the write has completed 
			break;
		} // ESP_GATTC_WRITE_DESCR_EVT

		default: {
			break;
		}
	} // End switch
}; // gattClientEventHandler

std::string BLERemoteDescriptor::readValue(void) {
	ESP_LOGD(LOG_TAG, ">> readValue: %s", toString().c_str());

	// Check to see that we are connected.
	if (!getRemoteCharacteristic()->getRemoteService()->getClient()->isConnected()) {
		ESP_LOGE(LOG_TAG, "Disconnected");
		throw BLEDisconnectedException();
	}

	m_semaphoreReadDescrEvt.take("readValue");

	// Ask the BLE subsystem to retrieve the value for the remote hosted characteristic.
	esp_err_t errRc = ::esp_ble_gattc_read_char_descr(
		m_pRemoteCharacteristic->getRemoteService()->getClient()->getGattcIf(),
		m_pRemoteCharacteristic->getRemoteService()->getClient()->getConnId(),    // The connection ID to the BLE server
		getHandle(),                                   // The handle of this characteristic
		ESP_GATT_AUTH_REQ_NONE);                       // Security

	if (errRc != ESP_OK) {
		ESP_LOGE(LOG_TAG, "esp_ble_gattc_read_char: rc=%d %s", errRc, GeneralUtils::errorToString(errRc));
		return "";
	}

	// Block waiting for the event that indicates that the read has completed.  When it has, the std::string found
	// in m_value will contain our data.
	m_semaphoreReadDescrEvt.wait("readValue");

	ESP_LOGD(LOG_TAG, "<< readValue(): length: %d", m_value.length());
	return m_value;
} // readValue


uint8_t BLERemoteDescriptor::readUInt8(void) {
	std::string value = readValue();
	if (value.length() >= 1) {
		return (uint8_t)value[0];
	}
	return 0;
} // readUInt8


uint16_t BLERemoteDescriptor::readUInt16(void) {
	std::string value = readValue();
	if (value.length() >= 2) {
		return *(uint16_t*)(value.data());
	}
	return 0;
} // readUInt16


uint32_t BLERemoteDescriptor::readUInt32(void) {
	std::string value = readValue();
	if (value.length() >= 4) {
		return *(uint32_t*)(value.data());
	}
	return 0;
} // readUInt32


/**
 * @brief Return a string representation of this BLE Remote Descriptor.
 * @retun A string representation of this BLE Remote Descriptor.
 */
std::string BLERemoteDescriptor::toString(void) {
	std::stringstream ss;
	ss << "handle: " << getHandle() << ", uuid: " << getUUID().toString();
	return ss.str();
} // toString


/**
 * @brief Write data to the BLE Remote Descriptor.
 * @param [in] data The data to send to the remote descriptor.
 * @param [in] length The length of the data to send.
 * @param [in] response True if we expect a response.
 */
void BLERemoteDescriptor::writeValue(
		uint8_t* data,
		size_t   length,
		bool     response) {
	ESP_LOGD(LOG_TAG, ">> writeValue: %s", toString().c_str());
	// Check to see that we are connected.
	if (!getRemoteCharacteristic()->getRemoteService()->getClient()->isConnected()) {
		ESP_LOGE(LOG_TAG, "Disconnected");
		throw BLEDisconnectedException();
	}

	esp_err_t errRc = ::esp_ble_gattc_write_char_descr(
		m_pRemoteCharacteristic->getRemoteService()->getClient()->getGattcIf(),
		m_pRemoteCharacteristic->getRemoteService()->getClient()->getConnId(),
		getHandle(),
		length,                           // Data length
		data,                             // Data
		ESP_GATT_WRITE_TYPE_NO_RSP,
		ESP_GATT_AUTH_REQ_NONE
	);
	if (errRc != ESP_OK) {
		ESP_LOGE(LOG_TAG, "esp_ble_gattc_write_char_descr: %d", errRc);
	}
	ESP_LOGD(LOG_TAG, "<< writeValue");
} // writeValue


/**
 * @brief Write data represented as a string to the BLE Remote Descriptor.
 * @param [in] newValue The data to send to the remote descriptor.
 * @param [in] response True if we expect a response.
 */
void BLERemoteDescriptor::writeValue(
		std::string newValue,
		bool        response) {
	writeValue(newValue.data(), newValue.length());
} // writeValue


/**
 * @brief Write a byte value to the Descriptor.
 * @param [in] The single byte to write.
 * @param [in] True if we expect a response.
 */
void BLERemoteDescriptor::writeValue(
		uint8_t newValue,
		bool    response) {
	writeValue(&newValue, 1, response);
} // writeValue


#endif /* CONFIG_BT_ENABLED */
