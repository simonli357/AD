#include "Database.hpp"
#include <ros/init.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "database_node");
	Database db;
	db.print_tables();
	ros::spin();
}
