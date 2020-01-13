//Class for declaring default values of zmq subscriber socket
#include<zmq.h>
#include"zhelpers.hpp"


class sub_socket
{
    public :

        zmq::context_t sub_context;

        zmq::socket_t subscriber;

    sub_socket();

    void bridge_subscribe();
    
};