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
#include "wsg_50_common/Move.h"
#include "std_srvs/Empty.h"


float forceThreshold = 4.0;
float currentEffortMaster = 0.0,
      currentEffortSlave = 0.0;
float currentPos;

ros::ServiceClient moveClientMaster, moveClientSlave,
                   stopClientMaster, stopClientSlave;
ros::Subscriber wsgStateSubMaster, wsgStateSubSlave;
int counter[2];
bool use_fmf,
     use_feedback;

void moveCallbackMaster(const wsg_50_common::Status::ConstPtr& msg)
{
    currentPos = msg->width;
    if (use_fmf)
        currentEffortMaster = msg->force_finger0 + msg->force_finger1;
    else
        currentEffortMaster = msg->force;
    counter[0]++;
}

void moveCallbackSlave(const wsg_50_common::Status::ConstPtr& msg)
{
    if (use_fmf)
        currentEffortSlave = msg->force_finger0 + msg->force_finger1;
    else
        currentEffortSlave = msg->force;
    counter[1]++;
}

void controlThread()
{
    ROS_INFO("Control Thread Started");
    
    float deltaTime = 0.1;
    float targetPos, targetVel;
    float k = 1.0;
    
    wsg_50_common::Move srv;
    std_srvs::Empty srvStop;
    
    while (ros::ok())
    {
        if ((counter[0] > 0) && (counter[1] > 0))
        {
            if ((fabs(currentEffortMaster) > forceThreshold) || (fabs(currentEffortSlave) > forceThreshold))
            {                
                targetPos = currentPos + (currentEffortMaster + currentEffortSlave) * k * deltaTime;
                targetVel = fabs((currentEffortMaster + currentEffortSlave) * k);
            
//                ROS_INFO("1 Pos %f, Vel %f, Force %f", targetPos, targetVel, currentForce);
                srv.request.width = targetPos;
                srv.request.speed = targetVel;
                moveClientMaster.call(srv);
                moveClientSlave.call(srv);
                
                ros::Duration(deltaTime).sleep();
                counter[0] = counter[1] = 0;
            }
            else
            {
                stopClientMaster.call(srvStop);
                stopClientSlave.call(srvStop);
                counter[0] = counter[1] = 0;
            }
        }    
    }          
    ROS_INFO("Control Thread Ended");
}

int main( int argc, char **argv )
{
    ros::init(argc, argv, "wsg_force_control");
    ros::NodeHandle n;
    
    n.param("use_fmf", use_fmf, false);    
    n.param("use_feedback", use_feedback, true);
    
    wsgStateSubMaster = n.subscribe("/wsg_50_driver_master/status", 1000, moveCallbackMaster);
    if (use_feedback)
        wsgStateSubSlave = n.subscribe("/wsg_50_driver_slave/status", 1000, moveCallbackSlave);   
    
    moveClientMaster = n.serviceClient<wsg_50_common::Move>("/wsg_50_driver_master/move_no_waiting");
    stopClientMaster = n.serviceClient<std_srvs::Empty>("/wsg_50_driver_master/stop_no_waiting");

    moveClientSlave = n.serviceClient<wsg_50_common::Move>("/wsg_50_driver_slave/move_no_waiting");
    stopClientSlave = n.serviceClient<std_srvs::Empty>("/wsg_50_driver_slave/stop_no_waiting");
   
    counter[0] = 0; counter[1] = 0;
    currentPos = 110.0;     //Referenced
   
    std::thread controlLoop;
    controlLoop = std::thread(controlThread);

    ros::spin();
	return 0;

}
