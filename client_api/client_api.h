#ifndef __ARM_CONTROL_API_H__
#define __ARM_CONTROL_API_H__

#define ERR_OK               (0)
#define ERR_NEW_CMD_FAIL     (-1)
#define ERR_SEND_CMD_FAIL    (-2)
#define ERR_RECV_CMD_TIMEOUT (-3)
#define ERR_RECV_CMD_FAILED  (-4)


#define MAX_JOINT_NUM (6)

#define JOINT_CTL_STATUS_IDLE (0)
#define JOINT_CTL_STATUS_BUSY (1)

#define ARM_STATUS_OK     (0)
#define ARM_STATUS_ERROR  (1)

#define GRIPPER_STATE_STOP (0)
#define GRIPPER_STATE_OPENNING (1)
#define GRIPPER_STATE_CLOSING (2)

#define JOINT_ACTION_NO_FINISHED  (0)
#define JOINT_ACTION_FINISHED     (1)

typedef  struct
{
    double pos[MAX_JOINT_NUM];   /* 六个关节位置，单位deg */
} joint_pos_t;


//connect the arm using ip address
inline int connect_arm(const char *server_ip);

//disconnect the arm
inline void disconnect_arm(void);

// get the status of the ARM
inline int get_arm_status(int *arm_status);

// get the current position of the joints of the ARM
inline int get_joint_pos(joint_pos_t *j_pos);

// set the target position of the joints of the ARM
inline unsigned int set_joint_pos(const joint_pos_t * j_pos, unsigned int execute_time_us);

// get the status of the jonint action
inline int get_joint_action_status(unsigned int action_index);

// get the control state of the gripper
inline int get_gripper_state();

/**
 * @brief Set the control state of the gripper
 * 
 * @param gripper_state 0 to stop, 1 to open, 2 to close.
 * @return int 
 */
inline int set_gripper_state(int gripper_state);


#endif
