
//ros dependencies
#include "image_distortion_node.h"

//node main
int main(int argc, char **argv)
{
    //init ros
    ros::init(argc, argv, "image_distortion");
    
    //create ros wrapper object
    ImageDistortionNode idn;
    
    //set node loop rate
    ros::Rate loop_rate(idn.rate());

    //main loop
    while ( ros::ok() ) 
    {
        //execute pending callbacks
        ros::spinOnce(); 
            
        //just publish the cloud
        idn.publish(); 
        
        //relax to fit output rate
        loop_rate.sleep();            
    }

    //exit program
    return 0;
}
