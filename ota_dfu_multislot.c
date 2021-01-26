#include <ota_dfu_multislot.h>
#include "gatt_db.h"
#include "native_gecko.h"
#include "btl_interface.h"
#include "btl_interface_storage.h"

// OTA DFU block size
#define SWITCHED_MULTIPROTOCOL_OTA_DFU_START        0x00
#define SWITCHED_MULTIPROTOCOL_OTA_DFU_COMMIT       0x03

uint32_t ota_data_len;

#define MAX_SLOT_ID 0xFFFFFFFF
static uint32_t otaDfuOffset;
static uint32_t otaDfuSlotId = MAX_SLOT_ID;
//static uint8_t conn_handle = 0;

uint8_t bleOtaDfuTransactionBegin(uint8_t len, uint8_t *data);
uint8_t bleOtaDfuTransactionFinish(uint32_t totalLen);
uint8_t bleOtaDfuDataReceived(uint8_t len, uint8_t *data);

void ota_dfu_handle_event(struct gecko_cmd_packet* evt)
{
  // Do not try to process NULL event.
  if (NULL == evt) {
    return;
  }

  // Handle events
  switch (BGLIB_MSG_ID(evt->header)) {

    case  gecko_evt_system_boot_id:

    		bootloader_init();

    	break;

    case  gecko_evt_le_connection_opened_id:

      //conn_handle = evt->data.evt_le_connection_opened.connection;
        
        //set timeout to 10 sec, because erasing the slot may take several seconds!!!
   	  //gecko_cmd_le_connection_set_parameters(conn_handle,6,6,0,1000);

    	break;

    case  gecko_evt_gatt_server_user_write_request_id:{

      uint32_t  connection      = evt->data.evt_gatt_server_user_write_request.connection;
      uint32_t  characteristic  = evt->data.evt_gatt_server_user_write_request.characteristic;
      uint8_t   *charValueData  = evt->data.evt_gatt_server_user_write_request.value.data;

      uint8_t writeRequestAttOpcode = evt->data.evt_gatt_server_user_write_request.att_opcode;
      uint8_t charLen = evt->data.evt_gatt_server_user_write_request.value.len;

      /*****  OTA Control  *****/
      if (characteristic == gattdb_ota_control) {
    	OTAErrorCodes_t responseVal = OTA_NoError;
        uint8_t otaDfuCommand = charValueData[0];
        uint8_t *otaDfuCommandVal = (uint8_t*)&charValueData[1];
        uint8_t otaDfuCommandValLen = (0 == charLen) ? (charLen) : (charLen - 1);

        switch (otaDfuCommand) {
          case SWITCHED_MULTIPROTOCOL_OTA_DFU_START:
              ota_data_len = 0;
              responseVal = bleOtaDfuTransactionBegin(otaDfuCommandValLen, otaDfuCommandVal);
            break;

          case SWITCHED_MULTIPROTOCOL_OTA_DFU_COMMIT:
        	  responseVal = bleOtaDfuTransactionFinish(ota_data_len);
            break;

          default:
              //responseVal = OTA_WrongValueError;
        	  gecko_cmd_gatt_server_send_user_write_response(connection, characteristic, OTA_NoError);
        	  gecko_cmd_le_connection_close(evt->data.evt_gatt_server_user_write_request.connection);
            break;
        }

        gecko_cmd_gatt_server_send_user_write_response(connection, characteristic, responseVal);

      } else

      /*****  OTA Data  *****/
      if (characteristic == gattdb_ota_data) {
        OTAErrorCodes_t responseVal = OTA_NoError;

        ota_data_len += charLen;

        responseVal = bleOtaDfuDataReceived(charLen, charValueData);

        // Send back response in case of data was sent as a write request
        if (BLE_OPCODE_GATT_WRITE_REQUEST == writeRequestAttOpcode) {
          gecko_cmd_gatt_server_send_user_write_response(connection, characteristic, responseVal);

        }
      }

      break;
    }

    // Value of attribute changed from the local database by remote GATT client
    case gecko_evt_gatt_server_attribute_value_id:

      /*****  OTA Data  *****/
      if (evt->data.evt_gatt_server_attribute_value.attribute == gattdb_ota_data) {
        uint8_t *charValueData = evt->data.evt_gatt_server_attribute_value.value.data;

        uint8_t len = evt->data.evt_gatt_server_user_write_request.value.len;

        ota_data_len += len;

        bleOtaDfuDataReceived(len, charValueData);
      } else
      /***** Boot slot *****/
      if (evt->data.evt_gatt_server_attribute_value.attribute == gattdb_boot_slot) {

    	  if (BOOTLOADER_OK == bootloader_setImageToBootload(evt->data.evt_gatt_server_attribute_value.value.data[0])) {
    	      bootloader_rebootAndInstall();
    	    }
      }

      break;

    default:
      break;
  }
}


uint8_t bleOtaDfuTransactionBegin(uint8_t len, uint8_t *data)
{
  int32_t rv;

  struct gecko_msg_gatt_server_read_attribute_value_rsp_t* res = gecko_cmd_gatt_server_read_attribute_value(gattdb_upload_slot,0);

  otaDfuSlotId = res->value.data[0];

  otaDfuOffset = 0;

  //erasing slot. This may take several seconds!!!
  rv = bootloader_eraseStorageSlot(otaDfuSlotId);

  return (BOOTLOADER_OK == rv) ? OTA_NoError : OTA_EraseError;
}

uint8_t bleOtaDfuTransactionFinish(uint32_t totalLen)
{
  int8_t ret = OTA_NoError;

  otaDfuSlotId = MAX_SLOT_ID;

  return ret;
}

uint8_t bleOtaDfuDataReceived(uint8_t len, uint8_t *data)
{
  int32_t rv;
  BootloaderStorageSlot_t storageSlot;

  rv = bootloader_getStorageSlotInfo(otaDfuSlotId, &storageSlot);
  if (BOOTLOADER_OK != rv) {
    return OTA_SlotError;
  }

  if ((otaDfuOffset + len) > storageSlot.length) {
	return OTA_SizeError;
  }

  rv = bootloader_writeStorage(otaDfuSlotId, otaDfuOffset, data, len);

  if (BOOTLOADER_OK == rv) {
    otaDfuOffset += len;
    return OTA_NoError;
  }
  return OTA_WriteError;

}
