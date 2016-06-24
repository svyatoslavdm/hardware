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
#include "std_srvs/Empty.h"
#include "std_msgs/Float32.h"


float forceThreshold = 4.0;
float currentForce;
float currentPos;

ros::ServiceClient moveClient,
                   stopClient;
ros::Publisher wsgCmdPub;
bool use_fmf;

void moveCallback(const wsg_50_common::Status::ConstPtr& msg)
{
    if (use_fmf)
        currentForce = msg->force_finger0 + msg->force_finger1;
    else
        currentForce = msg->force;
    currentPos = msg->width;
}

void controlThread()
{
    ROS_INFO("Control Thread Started");
    
    float deltaTime = 0.05;
    float targetPos, targetVel;
    float k = 1;
    
    std_msgs::Float32 msg;
    
    while (ros::ok())
    {
        if (fabs(currentForce) > forceThreshold)
        {                
            targetPos = currentPos + currentForce * k * deltaTime;
            targetVel = currentForce * k;
            msg.data = targetVel;
            wsgCmdPub.publish(msg);
            ros::Duration(deltaTime).sleep();
        }    
        else
        {   
            msg.data = 0.0;
            wsgCmdPub.publish(msg);
            ros::Duration(deltaTime).sleep();
        }
    }    
    ROS_INFO("Control Thread Ended");
}

int main( int argc, char **argv )
{
    ros::init(argc, argv, "wsg_force_control");
    ros::NodeHandle n;

    n.param("use_fmf", use_fmf, true);
    
    ROS_INFO("Communication mode ethernet (auto_update required)");
    wsgCmdPub = n.advertise<std_msgs::Float32>("/wsg_50_driver/goal_speed", 40);

    ros::Subscriber wsgStateSub = n.subscribe("/wsg_50_driver/status", 1000, moveCallback);
    
    std::thread controlLoop;
    controlLoop = std::thread(controlThread);

    ros::spin();
	return 0;

}
