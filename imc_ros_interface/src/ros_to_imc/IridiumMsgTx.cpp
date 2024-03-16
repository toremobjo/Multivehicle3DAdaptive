// LICENCE
#include <imc_ros_bridge/ros_to_imc/IridiumMsgTx.h>
#include <IMC/Base/Serialization.hpp>

namespace ros_to_imc {

template <>
bool convert(const imc_ros_interface::IridiumMsgTx& ros_msg, IMC::IridiumMsgTx& imc_msg)
{
    imc_msg.req_id = ros_msg.req_id.data;
    imc_msg.ttl = ros_msg.ttl.data;
    imc_msg.destination = ros_msg.destination.data;
    std::vector<signed char> array = ros_msg.data.data;
    std::vector<char> vec(array.begin(), array.end());
    //std::cout << "Sending iridium message: " << str << std::endl;

    //std::cout << "Sending iridium message: " << str.end() << std::endl;
    //std::cout << "Sending iridium message: " << vec.length() << std::endl;
    /*
    uint16_t len = vec.size()+2;
    uint8_t a = len%256;
    uint8_t b = floor(len/256);
    //vec[0] = b;
    //vec[1] = a;
    vec.insert(vec.begin(),a);
    vec.insert(vec.begin(),b);

    std::cout << a << std::endl;
    std::cout << b << std::endl;
    std::cout << len << std::endl;

    int hh = 0;
    for (int i:vec){
      std::ostringstream ss;
      ss << std::hex << i;
      std::string result = ss.str();
      std::cout << result << " ";//std::vector<char> vecc(len,vec);
      hh += 1;
    }*/
    imc_msg.data = vec;

    return true;
}

} // namespace imc_to_ros
