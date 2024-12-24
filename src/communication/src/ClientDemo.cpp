#include "ros/time.h"
#include <Client.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <unistd.h>

sensor_msgs::Image create_fake_image(int width, int height) {
	// Create an empty image
	sensor_msgs::Image img;

	// Use std::chrono to generate a timestamp (in seconds)
	auto now = std::chrono::system_clock::now();
	auto duration = now.time_since_epoch();
	auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

	img.header.stamp = ros::Time(timestamp / 1000.0); // Convert to ros::Time
	img.header.frame_id = "camera_frame";			  // Frame of reference
	img.height = height;
	img.width = width;
	img.encoding = sensor_msgs::image_encodings::RGB8; // RGB image encoding
	img.is_bigendian = true;						   // Little endian
	img.step = width * 3;							   // 3 bytes per pixel for RGB8 encoding
	img.data.resize(width * height * 3);			   // Total number of bytes

	// Fill the image with some test data (e.g., alternating red and green pixels)
	for (int i = 0; i < width * height; ++i) {
		// Example: RGB pattern with alternating red and green pixels
		if (i % 2 == 0) {
			img.data[i * 3] = 255;	 // Red (R)
			img.data[i * 3 + 1] = 0; // Green (G)
			img.data[i * 3 + 2] = 0; // Blue (B)
		} else {
			img.data[i * 3] = 0;	   // Red (R)
			img.data[i * 3 + 1] = 255; // Green (G)
			img.data[i * 3 + 2] = 0;   // Blue (B)
		}
	}

	return img;
}

int main(int argc, char *argv[]) {
	Client client("127.0.0.1", 49153, 10485760);
	client.initialize();
	client.send_string("TEST"); // String test
	client.send_image(create_fake_image(800, 800));
	while (true) {
		if (!client.get_strings().empty()) {
			std::cout << client.get_strings().front() << std::endl;
			client.get_strings().pop();
		}
		sleep(1);
	}
	return 0;
}
