MODULE ROS_motion_left

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

LOCAL CONST zonedata DEFAULT_CORNER_DIST := fine;
LOCAL VAR ROS_msg_joint_data local_target;
LOCAL VAR intnum intr_new_target;

PROC main()
    VAR jointtarget target;
    VAR speeddata move_speed := v100;  ! default speed
    VAR zonedata stop_mode;
    VAR bool skip_move;
    
    ! Set up interrupt to watch for new trajectory
    !IDelete intr_new_target;    ! clear interrupt handler, in case restarted with ExitCycle
    !CONNECT intr_new_target WITH new_target_handler;
    !IPers ROS_joint_target_left_lock, intr_new_target;

    WHILE true DO
        ! Check for new Trajectory
        WaitTestAndSet ROS_joint_target_left_lock; ! wait for a the mutex to copy over
        IF (ROS_new_joint_target_left) THEN ! wait for a new setpoint
            local_target := next_joint_target;            ! copy to local var
                       
            ROS_new_joint_target_left := FALSE;
            ROS_joint_target_left_lock := FALSE;        ! release data-lock
            
            ! execute all points in this trajectory
            target.robax := local_target.joints_left.robax;
            target.extax.eax_a := local_target.joints_left.extax.eax_a;
            stop_mode := DEFAULT_CORNER_DIST;  ! assume we're smoothing between points
            MoveAbsJ target, move_speed, stop_mode, tool0;

            
        ELSE
            ROS_joint_target_left_lock := FALSE;        ! release data-lock
            WaitTime 0.005;
        ENDIF        
    ENDWHILE
ERROR
    ErrWrite \W, "Motion Error", "Error executing motion.  Aborting trajectory.";
    abort_trajectory;
ENDPROC

!LOCAL PROC init_target()
!    clear_path;                    ! cancel any active motions

!    WaitTestAndSet ROS_joint_target_left_lock;  ! acquire data-lock
!    local_target := next_joint_target;            ! copy to local var
!    ROS_new_joint_target_left := FALSE;         
!    ROS_joint_target_left_lock := FALSE;        ! release data-lock
!ENDPROC

LOCAL PROC abort_trajectory()
    clear_path;
    ExitCycle;  ! restart program
ENDPROC

LOCAL PROC clear_path()
    IF ( NOT (IsStopMoveAct(\FromMoveTask) OR IsStopMoveAct(\FromNonMoveTask)) )
        StopMove;          ! stop any active motions
    ClearPath;             ! clear queued motion commands
    StartMove;             ! re-enable motions
ENDPROC

!LOCAL TRAP new_target_handler
!    IF (NOT ROS_new_trajectory) RETURN;
    
!    abort_trajectory;
!ENDTRAP

ENDMODULE
