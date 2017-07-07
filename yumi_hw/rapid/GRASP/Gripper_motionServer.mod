MODULE ROS_motionServer

! Software License Agreement (BSD License)
!
! Copyright (c) 2012, Edward Venator, Case Western Reserve University
! Copyright (c) 2012, Jeremy Zoss, Southwest Research Institute
! All rights reserved.
!
! Redistribution and use in source and binary forms, with or without modification,
! are permitted provided that the following conditions are met:
!
!   Redistributions of source code must retain the above copyright notice, this
!       list of conditions and the following disclaimer.
!   Redistributions in binary form must reproduce the above copyright notice, this
!       list of conditions and the following disclaimer in the documentation
!       and/or other materials provided with the distribution.
!   Neither the name of the Case Western Reserve University nor the names of its contributors
!       may be used to endorse or promote products derived from this software without
!       specific prior written permission.
!
! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
! EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
! OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
! SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
! INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
! TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
! BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
! CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
! WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

LOCAL CONST num server_port := 12000;

LOCAL VAR socketdev server_socket;
LOCAL VAR socketdev client_socket;


PROC main()
    VAR ROS_msg_gripper_target message;

    TPWrite "GripperMotionServer: Waiting for connection.";
	ROS_init_socket server_socket, server_port;
    ROS_wait_for_client server_socket, client_socket;

    WHILE ( true ) DO
		! Recieve Gripper Trajectory Pt Message
        ROS_receive_msg_gripper_data client_socket, message;
        WaitTestAndSet ROS_gripper_left_lock;
        ROS_new_gripper_left := TRUE;
        next_grasp_target.left := message.left;
        ROS_gripper_left_lock := FALSE;
        
        WaitTestAndSet ROS_gripper_right_lock;
        ROS_new_gripper_right := TRUE;
        next_grasp_target.right := message.right;
        ROS_gripper_right_lock := FALSE;
        
	ENDWHILE

ERROR (ERR_SOCK_TIMEOUT, ERR_SOCK_CLOSED)
	IF (ERRNO=ERR_SOCK_TIMEOUT) OR (ERRNO=ERR_SOCK_CLOSED) THEN
        SkipWarn;  ! TBD: include this error data in the message logged below?
        ErrWrite \W, "ROS MotionServer disconnect", "Connection lost.  Resetting socket.";
		ExitCycle;  ! restart program
	ELSE
		TRYNEXT;
	ENDIF
UNDO
	IF (SocketGetStatus(client_socket) <> SOCKET_CLOSED) SocketClose client_socket;
	IF (SocketGetStatus(server_socket) <> SOCKET_CLOSED) SocketClose server_socket;
ENDPROC

	
ENDMODULE
