/*
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <ros/ros.h>
#include <thread>
#include <cmath>

#include "wsg_50_common/Status.h"
#include "wsg_50_common/Cmd.h"
#include "std_msgs/Float32.h"

float currentEffortMaster = 0.0,
      currentEffortSlave = 0.0;
float currentPosMaster,
      currentPosSlave;     
float prevEffortMaster,
      prevEffortSlave;

double forceThreshold;
double loopRate;

ros::Publisher wsgCmdPubMaster, wsgCmdPubSlave, wsgCmdPubSlavePosition;
ros::Subscriber wsgStateSubMaster, wsgStateSubSlave;
     
void moveCallbackMaster(const wsg_50_common::Status::ConstPtr& msg)
{
    currentPosMaster = msg->width;
    currentEffortMaster = msg->force_finger0 + msg->force_finger1;
}

void moveCallbackSlave(const wsg_50_common::Status::ConstPtr& msg)
{
    currentPosSlave = msg->width;
    currentEffortSlave = msg->force_finger0 + msg->force_finger1;
}

void controlThread()
{
    ROS_INFO("Control Thread Started");
    
    float deltaTime = 1.0 / loopRate;
    float targetVel;
    float k1 = 1.0,
          k2 = 0.3;
    float currentSumForce, prevSumForce;
    
    std_msgs::Float32 msg;
    
    prevSumForce = currentSumForce = currentEffortMaster + currentEffortSlave;
    
    while (ros::ok())
    {        
        if (fabs(currentPosMaster - currentPosSlave) > 10.0)
        {
            ROS_WARN("Gripper fault!");
            wsg_50_common::Cmd msg_pos;
            msg_pos.pos = currentPosMaster;
            msg_pos.speed = 5.0;
//             wsgCmdPubSlavePosition.publish(msg_pos);
//            msg.data = 0.0;
//            wsgCmdPubMaster.publish(msg);
//            wsgCmdPubSlave.publish(msg);  
//            return;
        }
        if ((fabs(currentEffortMaster) > forceThreshold) || (fabs(currentEffortSlave) > forceThreshold))
        {       
            currentSumForce = currentEffortMaster + currentEffortSlave;
            targetVel = currentSumForce * k1 - (currentSumForce - prevSumForce) * k2;   
            
            msg.data = targetVel;
            wsgCmdPubMaster.publish(msg);
            wsgCmdPubSlave.publish(msg);            
            ros::Duration(deltaTime).sleep();
        } 
        else
        {
            msg.data = 0.0;//0.001;
            wsgCmdPubMaster.publish(msg);
            wsgCmdPubSlave.publish(msg);  
            ros::Duration(deltaTime).sleep();
        }
        prevSumForce = currentSumForce;
    }    
    ROS_INFO("Control Thread Ended");
}

int main( int argc, char **argv )
{
    ros::init(argc, argv, "wsg_force_control");
    ros::NodeHandle n;
    
    n.param("loop_rate", loopRate, 50.0);
    n.param("force_threshold", forceThreshold, 5.0);
        
    wsgStateSubMaster = n.subscribe("/wsg_50_driver_master/status", 1000, moveCallbackMaster);
    wsgStateSubSlave = n.subscribe("/wsg_50_driver_slave/status", 1000, moveCallbackSlave);  
    ROS_INFO("Control loop rate: %f", loopRate);
    ROS_INFO("Force Threshold: %f", forceThreshold);
    ROS_INFO("Force measure fingers required");
        
    wsgCmdPubMaster = n.advertise<std_msgs::Float32>("/wsg_50_driver_master/goal_speed", 10);
    wsgCmdPubSlave = n.advertise<std_msgs::Float32>("/wsg_50_driver_slave/goal_speed", 10);
    wsgCmdPubSlavePosition = n.advertise<wsg_50_common::Cmd>("/wsg_50_driver_slave/goal_position", 10);
    
    std::thread controlLoop;
    controlLoop = std::thread(controlThread);

    ros::spin();
	return 0;

}
