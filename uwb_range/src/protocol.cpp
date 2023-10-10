/**
 * @file protocol.cpp
 * @author Vit Petrik (petrivi2@fel.cvut.cz)
 * @brief  Serialization and deserialization of protocol used to communicate between nRF and ROS
 * @version 0.1
 * @date 2022-12-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "protocol.h"

/**
 * @brief serialize ros_msg_t to baca protocol struct
 * 
 * @param msg reference to an struct ros_msg_t
 * @param baca refernce to baca protocol structure
 */
void serialize_ros(const struct ros_msg_t *msg, mrs_modules_msgs::BacaProtocol &baca)
{
    baca.payload.push_back(msg->address);
    baca.payload.push_back(msg->mode);

    if (msg->mode == 'r')
        return;

    switch (msg->address)
    {
    case WHO_I_AM:
        baca.payload.insert(baca.payload.end(),
                            msg->data.id_msg.id,
                            msg->data.id_msg.id + sizeof(msg->data.id_msg.id));
        break;
    case TRX_DATA:
        baca.payload.insert(baca.payload.end(),
                    reinterpret_cast<const char*>(&msg->data.uwb_data_msg.source_mac),
                    reinterpret_cast<const char*>(&msg->data.uwb_data_msg.source_mac) + sizeof(msg->data.uwb_data_msg.source_mac));
        baca.payload.insert(baca.payload.end(),
                    reinterpret_cast<const char*>(&msg->data.uwb_data_msg.destination_mac),
                    reinterpret_cast<const char*>(&msg->data.uwb_data_msg.destination_mac) + sizeof(msg->data.uwb_data_msg.destination_mac));
        baca.payload.insert(baca.payload.end(),
                    reinterpret_cast<const char*>(&msg->data.uwb_data_msg.msg_type),
                    reinterpret_cast<const char*>(&msg->data.uwb_data_msg.msg_type) + sizeof(msg->data.uwb_data_msg.msg_type));
        baca.payload.insert(baca.payload.end(),
                    reinterpret_cast<const char*>(&msg->data.uwb_data_msg.payload_size),
                    reinterpret_cast<const char*>(&msg->data.uwb_data_msg.payload_size) + sizeof(msg->data.uwb_data_msg.payload_size));
        baca.payload.insert(baca.payload.end(),
                    reinterpret_cast<const char*>(msg->data.uwb_data_msg.payload),
                    reinterpret_cast<const char*>(msg->data.uwb_data_msg.payload) + msg->data.uwb_data_msg.payload_size);
        break;
    case RESET:
        baca.payload.insert(baca.payload.end(),
                    reinterpret_cast<const char*>(&msg->data.reset),
                    reinterpret_cast<const char*>(&msg->data.reset) + sizeof(msg->data.reset));
        break;
    case ROS_CONTROL:
        baca.payload.insert(baca.payload.end(),
            reinterpret_cast<const char*>(&msg->data.control),
            reinterpret_cast<const char*>(&msg->data.control) + sizeof(msg->data.control));
        break;
    case REQUEST_RANGING:
        baca.payload.insert(baca.payload.end(),
            reinterpret_cast<const char*>(&msg->data.request_ranging.target_id),
            reinterpret_cast<const char*>(&msg->data.request_ranging.target_id) + sizeof(msg->data.request_ranging.target_id));
        baca.payload.insert(baca.payload.end(),
            reinterpret_cast<const char*>(&msg->data.request_ranging.preprocessing),
            reinterpret_cast<const char*>(&msg->data.request_ranging.preprocessing) + sizeof(msg->data.request_ranging.preprocessing));
        break;
    case RANGING_RESULT:
    default:
        // nothing to do here
        break;
    }
}

/**
 * @brief deserialize baca protocol msg to ros_msg_t
 * 
 * @param msg reference to an struct ros_msg_t
 * @param baca refernce to baca protocol structure
 */
void deserialize_ros(struct ros_msg_t *msg, const mrs_modules_msgs::BacaProtocol &baca)
{
    int index = 0;

    msg->address = baca.payload[index++];
    msg->mode = baca.payload[index++];

    if (msg->mode == 'r')
        return;

    switch (msg->address)
    {
    case WHO_I_AM:
        std::copy(baca.payload.begin() + index,
                  baca.payload.begin() + index + sizeof(msg->data.id_msg.id),
                  &msg->data.id_msg.id[0]);
        break;
    case TRX_DATA:
        std::copy(baca.payload.begin() + index,
                baca.payload.begin() + index + sizeof(msg->data.uwb_data_msg.source_mac),
                reinterpret_cast<char*>(&msg->data.uwb_data_msg.source_mac));
        index += sizeof(msg->data.uwb_data_msg.source_mac);
        std::copy(baca.payload.begin() + index,
                baca.payload.begin() + index + sizeof(msg->data.uwb_data_msg.destination_mac),
                reinterpret_cast<char*>(&msg->data.uwb_data_msg.destination_mac));
        index += sizeof(msg->data.uwb_data_msg.destination_mac);
        std::copy(baca.payload.begin() + index,
                baca.payload.begin() + index + sizeof(msg->data.uwb_data_msg.msg_type),
                reinterpret_cast<char*>(&msg->data.uwb_data_msg.msg_type));
        index += sizeof(msg->data.uwb_data_msg.msg_type);
        std::copy(baca.payload.begin() + index,
                baca.payload.begin() + index + sizeof(msg->data.uwb_data_msg.payload_size),
                reinterpret_cast<char*>(&msg->data.uwb_data_msg.payload_size));
        index += sizeof(msg->data.uwb_data_msg.payload_size);
        std::copy(baca.payload.begin() + index,
                baca.payload.begin() + index + msg->data.uwb_data_msg.payload_size,
                reinterpret_cast<char*>(&msg->data.uwb_data_msg.payload));
        index += msg->data.uwb_data_msg.payload_size;
        break;
    case RANGING_RESULT:
        std::copy(baca.payload.begin() + index,
                  baca.payload.begin() + index + sizeof(msg->data.ranging_msg.source_mac),
                  reinterpret_cast<char*>(&msg->data.ranging_msg.source_mac));
        index += sizeof(msg->data.ranging_msg.source_mac);
        std::copy(baca.payload.begin() + index,
                  baca.payload.begin() + index + sizeof(msg->data.ranging_msg.range),
                  reinterpret_cast<char*>(&msg->data.ranging_msg.range));
        index += sizeof(msg->data.ranging_msg.range);
        std::copy(baca.payload.begin() + index,
                  baca.payload.begin() + index + sizeof(msg->data.ranging_msg.variance),
                  reinterpret_cast<char*>(&msg->data.ranging_msg.variance));
        index += sizeof(msg->data.ranging_msg.variance);
        std::copy(baca.payload.begin() + index,
                  baca.payload.begin() + index + sizeof(msg->data.ranging_msg.raw),
                  reinterpret_cast<char*>(&msg->data.ranging_msg.raw));
        index += sizeof(msg->data.ranging_msg.raw);
        std::copy(baca.payload.begin() + index,
                  baca.payload.begin() + index + sizeof(msg->data.ranging_msg.power_a),
                  reinterpret_cast<char*>(&msg->data.ranging_msg.power_a));
        index += sizeof(msg->data.ranging_msg.power_a);
        std::copy(baca.payload.begin() + index,
                  baca.payload.begin() + index + sizeof(msg->data.ranging_msg.power_b),
                  reinterpret_cast<char*>(&msg->data.ranging_msg.power_b));
        index += sizeof(msg->data.ranging_msg.power_b);
        break;
    case RESET:
        std::copy(baca.payload.begin() + index,
                baca.payload.begin() + index + sizeof(msg->data.reset),
                reinterpret_cast<char*>(&msg->data.reset));
        index += sizeof(msg->data.reset);
        break;
    case ROS_CONTROL:
        std::copy(baca.payload.begin() + index,
            baca.payload.begin() + index + sizeof(msg->data.control),
            reinterpret_cast<char*>(&msg->data.control));
        index += sizeof(msg->data.control);
        break;
    default:
        break;
    }
}

/**
 * @brief deserialize baca protocol msg to anchor_msg_t
 * 
 * @param msg reference to an struct anchor_msg_t
 * @param baca refernce to baca protocol structure
*/
void deserialize_anchor_msg(struct anchor_msg_t *msg, uint8_t *buffer)
{
    int index = 0;

    msg->address = buffer[index++];
    msg->mode = buffer[index++];

    switch (msg->address)
    {
    case ANCHOR_BEACON:
        std::copy(buffer + index,
                buffer + index + sizeof(msg->data.anchor_beacon.capabilities),
                reinterpret_cast<char*>(&msg->data.anchor_beacon.capabilities));
        break;
    
    default:
        break;
    }
    return;
}
