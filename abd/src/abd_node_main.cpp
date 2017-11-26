#include "abd_node.cpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "abd_node");
  AbdNode node;

  ros::spin();
}

