#include <ActionServerBase.h>
#include <iostream>                                                                                 // std::cout

int main(int argc, char **argv)
{
     (void)argc;
     (void)argv;
     
     std::cout << "Worker bees can leave.\n";
     std::cout << "Even drones can fly away.\n";
     std::cout << "The Queen is their slave.\n";
     
     return 1;                                                                                      // No problems with main
}
