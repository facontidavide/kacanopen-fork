#include "device_rpdo.h"
#include "device_tpdo.h"
#include <cstdint>
void map_rpdo_in_device(RPDO_NO rpdo_no,
                        std::vector<uint32_t> entries_to_be_mapped,
                        uint8_t transmit_type, uint16_t inhibit_time,
                        uint16_t event_timer,
                        std::shared_ptr<kaco::Device> device, uint8_t node_id) {
  uint32_t cob_id = 0;
  switch (rpdo_no) {
    case rpdo1:
      std::cout << "Mapping RPDO1" << std::endl;
      // disable tpdo3
      cob_id = device->get_entry(0x1400, 01, kaco::ReadAccessMethod::sdo);
      cob_id ^= static_cast<uint32_t>((-1 ^ cob_id) & (1UL << 31));
      device->set_entry(0x1400, 0x01, static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      // delete no. of mapped entries
      device->set_entry(0x1600, 0x00, static_cast<uint8_t>(0x00),
                        kaco::WriteAccessMethod::sdo);
      // add new mapping
      write_entry(0x1600, entries_to_be_mapped, device);
      // update no. of mapped entries
      device->set_entry(0x1600, 0x00,
                        static_cast<uint8_t>(entries_to_be_mapped.size()),
                        kaco::WriteAccessMethod::sdo);
      // set transmit type
      device->set_entry(0x1400, 0x02, transmit_type,
                        kaco::WriteAccessMethod::sdo);
//      // set inhibit time
//      device->set_entry(0x1400, 0x03, inhibit_time,
//                        kaco::WriteAccessMethod::sdo);
//      // set event timer i.e transmit frequency
//      device->set_entry(0x1400, 0x05, event_timer,
//                        kaco::WriteAccessMethod::sdo);
      // enable tpdo1
      cob_id ^= static_cast<uint32_t>((-0 ^ cob_id) & (1UL << 31));
      device->set_entry(0x1400, 0x01, static_cast<uint32_t>(cob_id),
                        kaco::WriteAccessMethod::sdo);
      break;
    case rpdo2:
    std::cout << "Mapping RPDO2" << std::endl;
    // disable tpdo3
    device->set_entry(0x1801, 0x01, static_cast<uint32_t>(0x80000300+node_id),
                      kaco::WriteAccessMethod::sdo);
    // delete no. of mapped entries
    device->set_entry(0x1A01, 0x00, static_cast<uint8_t>(0x00),
                      kaco::WriteAccessMethod::sdo);
    // add new mapping
    write_entry(0x1A01, entries_to_be_mapped, device);
    // update no. of mapped entries
    device->set_entry(0x1A01, 0x00,
                      static_cast<uint8_t>(entries_to_be_mapped.size()),
                      kaco::WriteAccessMethod::sdo);
    // set transmit type
    device->set_entry(0x1801, 0x02, transmit_type,
                      kaco::WriteAccessMethod::sdo);
    // set inhibit time
    device->set_entry(0x1801, 0x03, inhibit_time,
                      kaco::WriteAccessMethod::sdo);
    // set event timer i.e transmit frequency
    device->set_entry(0x1801, 0x05, event_timer,
                      kaco::WriteAccessMethod::sdo);
    // enable tpdo1
    device->set_entry(0x1801, 0x01, static_cast<uint32_t>(0x00000300+node_id),
                      kaco::WriteAccessMethod::sdo);
      break;
    case rpdo3:
    std::cout << "Mapping RPDO3" << std::endl;
    // disable tpdo3
    device->set_entry(0x1802, 0x01, static_cast<uint32_t>(0x80000400+node_id),
                      kaco::WriteAccessMethod::sdo);
    // delete no. of mapped entries
    device->set_entry(0x1A02, 0x00, static_cast<uint8_t>(0x00),
                      kaco::WriteAccessMethod::sdo);
    // add new mapping
    write_entry(0x1A02, entries_to_be_mapped, device);
    // update no. of mapped entries
    device->set_entry(0x1A02, 0x00,
                      static_cast<uint8_t>(entries_to_be_mapped.size()),
                      kaco::WriteAccessMethod::sdo);
    // set transmit type
    device->set_entry(0x1802, 0x02, transmit_type,
                      kaco::WriteAccessMethod::sdo);
    // set inhibit time
    device->set_entry(0x1802, 0x03, inhibit_time,
                      kaco::WriteAccessMethod::sdo);
    // set event timer i.e transmit frequency
    device->set_entry(0x1802, 0x05, event_timer,
                      kaco::WriteAccessMethod::sdo);
    // enable tpdo1
    device->set_entry(0x1802, 0x01, static_cast<uint32_t>(0x00000400+node_id),
                      kaco::WriteAccessMethod::sdo);
      break;
    case rpdo4:
      std::cout << "Mapping RPDO4" << std::endl;
      // disable tpdo3
      device->set_entry(0x1803, 0x01, static_cast<uint32_t>(0x80000500+node_id),
                        kaco::WriteAccessMethod::sdo);
      // delete no. of mapped entries
      device->set_entry(0x1A03, 0x00, static_cast<uint8_t>(0x00),
                        kaco::WriteAccessMethod::sdo);
      // add new mapping
      write_entry(0x1A03, entries_to_be_mapped, device);
      // update no. of mapped entries
      device->set_entry(0x1A03, 0x00,
                        static_cast<uint8_t>(entries_to_be_mapped.size()),
                        kaco::WriteAccessMethod::sdo);
      // set transmit type
      device->set_entry(0x1803, 0x02, transmit_type,
                        kaco::WriteAccessMethod::sdo);
      // set inhibit time
      device->set_entry(0x1803, 0x03, inhibit_time,
                        kaco::WriteAccessMethod::sdo);
      // set event timer i.e transmit frequency
      device->set_entry(0x1803, 0x05, event_timer,
                        kaco::WriteAccessMethod::sdo);
      // enable tpdo3
      device->set_entry(0x1803, 0x01, static_cast<uint32_t>(0x00000500+node_id),
                        kaco::WriteAccessMethod::sdo);
      break;

    default:
      std::cout << "Maximum 4 RPDOs is supported" << std::endl;
      std::cout << "Invalid rpdo_no" << std::endl;
  }
}
