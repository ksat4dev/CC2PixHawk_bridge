//Class for declaring default values of zmq publisher socket
#include<zmq.h>
#include"zhelpers.hpp"


class pub_socket
{
    //zmq::context_t context;
   
    //zmq::socket_t publisher;
  
    //zmq::context_t sub_context;

    //zmq::socket_t subscriber;

public :

    zmq::context_t context;
    zmq::socket_t publisher;
    zmq::context_t sub_context;
    zmq::socket_t subscriber;
    
    pub_socket();   
};