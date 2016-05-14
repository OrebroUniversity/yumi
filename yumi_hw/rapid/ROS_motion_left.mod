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

LOCAL CONST zonedata DEFAULT_CORNER_DIST := z10;
LOCAL VAR ROS_msg_joint_data local_target;
LOCAL VAR intnum intr_new_target;

PROC main()
    VAR jointtarget target;
    VAR jointtarget prev_target;
    VAR bool first_target := TRUE;
    VAR speeddata move_speed := v100;  ! default speed
    VAR zonedata stop_mode;
    VAR bool skip_move;
    VAR clock clk;
    VAR num now;
    VAR num prev;
    VAR num local_cycle_time;
    VAR bool wasPosition := FALSE;
    
    ClkReset clk;
    ClkStart clk;

    prev := ClkRead(clk, \HighRes);
    now := ClkRead(clk, \HighRes);
    
    ! Set up interrupt to watch for new trajectory
    !IDelete intr_new_target;    ! clear interrupt handler, in case restarted with ExitCycle
    !CONNECT intr_new_target WITH new_target_handler;
    !IPers ROS_joint_target_left_lock, intr_new_target;

    prev_target :=CJointT();
    
    WHILE true DO
        ! Check for an updated setpoint. 
        IF (TestAndSet(ROS_joint_target_left_lock)) THEN ! mutex acquired, we can change the target
            IF (ROS_new_joint_target_left) THEN          ! a new setpoint is available
                local_target := next_joint_target;            ! copy to local var
            ENDIF
            ROS_new_joint_target_left := FALSE;
            ROS_joint_target_left_lock := FALSE;        ! release data-lock
        ENDIF

        !track setpoint position / velocity
        IF (local_target.mode=JOINT_POSITION) THEN
            target.robax:=local_target.joints_left.robax;
            target.extax.eax_a:=local_target.joints_left.extax.eax_a;
            wasPosition:=TRUE;
        ELSE
            IF (wasPosition) THEN
                prev_target :=CJointT();            
                wasPosition := FALSE;
            ENDIF
            !calculate next target from desired velocity
            now := ClkRead(clk, \HighRes);
            local_cycle_time := now - prev;
            target := compute_target(prev_target,local_target.joints_left,cycle_time);
            prev_target := target;
        ENDIF
                
        !check if we are too close to the target
        IF (NOT is_near(target,0.0001)) THEN
            !no, so we can move with a smoothing zone
            stop_mode:=DEFAULT_CORNER_DIST;                     
        ELSE
            !yes, we have to move with stoping
            stop_mode:=fine;        
        ENDIF
        prev := ClkRead(clk, \HighRes);
        MoveAbsJ target,move_speed,\T:=cycle_time,stop_mode,tool0;    
        
        
    ENDWHILE
ERROR
    ErrWrite \W, "Motion Error", "Error executing motion.  Aborting trajectory.";
    abort_trajectory;
ENDPROC


LOCAL FUNC jointtarget compute_target(jointtarget current, jointtarget vel, num cycle)
    VAR jointtarget ret;
    ret.robax.rax_1 := current.robax.rax_1 + vel.robax.rax_1*cycle;
    ret.robax.rax_2 := current.robax.rax_2 + vel.robax.rax_2*cycle;
    ret.robax.rax_3 := current.robax.rax_3 + vel.robax.rax_3*cycle;
    ret.robax.rax_4 := current.robax.rax_4 + vel.robax.rax_4*cycle;
    ret.robax.rax_5 := current.robax.rax_5 + vel.robax.rax_5*cycle;
    ret.robax.rax_6 := current.robax.rax_6 + vel.robax.rax_6*cycle;
    ret.extax.eax_a := current.extax.eax_a + vel.extax.eax_a*cycle;
    RETURN ret;
ENDFUNC

!LOCAL PROC init_target()
!    clear_path;                    ! cancel any active motions

!    WaitTestAndSet ROS_joint_target_left_lock;  ! acquire data-lock
!    local_target := next_joint_target;            ! copy to local var
!    ROS_new_joint_target_left := FALSE;         
!    ROS_joint_target_left_lock := FALSE;        ! release data-lock
!ENDPROC

LOCAL FUNC bool is_near(jointtarget target, num tol)
    VAR jointtarget curr_jnt;
    
    curr_jnt := CJointT();
    
    RETURN ( ABS(curr_jnt.robax.rax_1 - target.robax.rax_1) < tol )
       AND ( ABS(curr_jnt.robax.rax_2 - target.robax.rax_2) < tol )
       AND ( ABS(curr_jnt.robax.rax_3 - target.robax.rax_3) < tol )
       AND ( ABS(curr_jnt.robax.rax_4 - target.robax.rax_4) < tol )
       AND ( ABS(curr_jnt.robax.rax_5 - target.robax.rax_5) < tol )
       AND ( ABS(curr_jnt.robax.rax_6 - target.robax.rax_6) < tol )
       AND ( ABS(curr_jnt.extax.eax_a - target.extax.eax_a) < tol );
ENDFUNC

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
