#include <ctime>
#include <iostream>
#include <string>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;

//function to create string to be sent to client

std:string make_daytime_string(){
	using namespace std;
	time_t now = time(0); //save the current time
	return ctime(&now); //convert time to string and return
	
}

int main(){
	try{
		boost::asio::io_service io_service; //io_service needed for all asio libraries
		for(;;){
			tcp::socket socket(io_service);	//creates a socket to represent client
			accepter.accept(socket);	//waits for client
			
			std::string message = make_daytime_string(); //gets current time as string
			
			boost::system::error_code ignored_error;
			boost::asio::write(socket, boost::asio::buffer(message), ignored_error); //write to client
		
	}catch(std::exception& e){
		std::cerr << e.what() << endl;	//print exceptions
	}
	
	return 0;

}