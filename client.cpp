#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;

int main(int argc, char* argv[]){
	try{
		if (argc != 2){
			std::cerr << "Usage: client <host>" << std::endl;
			return 1;
		}
    	
    	boost::asio::io_service io_service;	//create asio io_service object 
    	
    	tcp::resolver resolver(io_service); 	//this turns the server name in arg 
    											//into a TCP endpoint
    											
    	tcp::resolver::query query(argv[1], "daytime"); //constructs a query for the server
    	
    	tcp::resolver::iterator endpoint_iterator = resolver.resolve(query); //list of tcp endpoints
    	
		tcp::socket socket(io_service);	//create the socket
		boost::asio::connect(socket, endpoint_iterator); //connect the client's socket with the server endpoint
		
		//the connection is now open
		
		for (;;){
			boost::array<char, 128> buf; //buffer array
			boost::system::error_code error; //for catching a broken connection
			size_t len = socket.read_some(boost::asio::buffer(buf), error); //for catching errors
    
			if (error == boost::asio::error::eof) //when the connection breaks
				break; // Connection closed cleanly by peer.
			else if (error) //if not eof, some other error has been found
				throw boost::system::system_error(error); // throw the error.

			std::cout.write(buf.data(), len);
		}
    }catch(std::exception& e){
    	std::cerr << e.what(); << std::endl; //print the exception
}