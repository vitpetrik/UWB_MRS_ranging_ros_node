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
void serialize_ros(const struct ros_msg_t *msg, mrs_msgs::BacaProtocol &baca)
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

    default:
        break;
    }
}

/**
 * @brief deserialize baca protocol msg to ros_msg_t
 * 
 * @param msg reference to an struct ros_msg_t
 * @param baca refernce to baca protocol structure
 */
void deserialize_ros(struct ros_msg_t *msg, const mrs_msgs::BacaProtocol &baca)
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
        break;
    default:
        break;
    }
}