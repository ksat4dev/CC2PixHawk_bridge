//
//	Pubsub envelope subscriber
//
#include "zhelpers.hpp"
#include<zmq.h>

int main() {
	// Prepare our context and subscriber
	zmq::context_t sub_context(1);
	zmq::socket_t subscriber (sub_context, ZMQ_SUB);
	subscriber.connect("tcp://192.168.1.3:3885");
	subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0);

/*	sub_socket::sub_socket() : sub_context(1), subscriber(sub_context,ZMQ_SUB) { 
    subscriber.connect("tcp://192.168.1.2:3885");
    subscriber.setsockopt(ZMQ_SUBSCRIBE,"",0);
}*/

/*void serialboost::SerialPort::bridge_subscribe()
{*/
    std::cout<<"Printing check"<<std::endl;
    try {
        while(1)
        {
            zmq::message_t pose_msgR;

            subscriber.recv(&pose_msgR);

            float x, y, z, quart_x, quart_y, quart_z, quart_w;
            
            std::istringstream iss(static_cast<char *>(pose_msgR.data()));

            iss>>x>>y>>z>>quart_x>>quart_y>>quart_z>>quart_w;

            std::cout<<"\n Position :\n"<<"\t"<<x<<"\n \t"<<y<<"\n \t"<<z<<std::endl;
            std::cout<<"\n Orientation :\n"<<"\t"<<quart_x<<"\n \t"<<quart_y<<"\n \t"<<quart_z<<"\n \t"<<quart_w<<std::endl;

            
        }
    }
    catch(std::exception &e){
        std::cout<<"Error Occurred!! \n";
    }
//}

/*	while(1)
	{
		// Read envelope with address
		std::string address = s_recv (subscriber);
		// Read message contents
		std::string contents = s_recv (subscriber);

		std::cout<<"[" << address << "]" << contents << std::endl;
	}
*/
	return 0;

}
