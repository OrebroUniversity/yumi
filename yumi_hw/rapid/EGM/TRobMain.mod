MODULE TRobMain
!======================================================================================================
! Software License Agreement (BSD License) 
!
! Copyright (c) 2015, ABB
! All rights reserved.
!
! Redistribution and use in source and binary forms, with
! or without modification, are permitted provided that 
! the following conditions are met:
!
!    * Redistributions of source code must retain the 
!      above copyright notice, this list of conditions 
!      and the following disclaimer.
!    * Redistributions in binary form must reproduce the 
!      above copyright notice, this list of conditions 
!      and the following disclaimer in the documentation 
!      and/or other materials provided with the 
!      distribution.
!    * Neither the name of ABB nor the names of its 
!      contributors may be used to endorse or promote 
!      products derived from this software without 
!      specific prior written permission.
!
! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
! AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
! IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
! ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
! LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
! DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
! SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
! CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
! OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF 
! THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
!======================================================================================================




    !==================================================================================================
    ! Primary components
    !==================================================================================================
    !-------------------------------------------------
    ! Data
    !-------------------------------------------------
    ! Flags indicating if:
    ! * IRB14000 is used, i.e. YuMi.
    ! * Synchronization should be used.
    VAR bool using_irb14000 := FALSE;
    VAR bool use_sync       := FALSE;
    
    ! Task actions.
    CONST num ACTION_IDLE                := 0;
    CONST num ACTION_GO_TO_HOME_POSITION := 1;
    CONST num ACTION_EGM                 := 2;
    VAR num current_action;    

    ! Error numbers.
    VAR errnum NEW_ACTION := 1;
    
    ! Interupt numbers.
    LOCAL VAR intnum interupt_go_to_home_position;
    LOCAL VAR intnum interupt_timer;
    
    ! Current joint values.
    LOCAL VAR jointtarget current_joints;
    
    !-------------------------------------------------
    ! Procedures
    !-------------------------------------------------
    PROC main()
        ! Initialize the main module. See comments in the procedure below
        ! for more information (e.g. how to trigger a new action).
        initializeMainModule;
        
        ! Initialize the Externally Guided Motion (EGM) module (e.g. interupt signals and EGM settings). 
        ! See comments in the procedure for more information.
        initializeEGMModule;
        
        ! Try to initialize the SmartGripper (SG) module (only tried if using IRB14000).
        IF using_irb14000 THEN
            CallByVar "initializeSGModule", 0;
        ENDIF

        ! handleGoToHomePositionAction;
        
        ! Manage the task's predefined actions.
        WHILE TRUE DO
            WaitTime 0.1;
            
            TEST current_action
                CASE ACTION_GO_TO_HOME_POSITION:
                    handleGoToHomePositionAction;
                    
                CASE ACTION_EGM:
                    handleEGMAction;
                    
                DEFAULT:
                    ! Do nothing.
            ENDTEST
        ENDWHILE
        
        !-------------------------------------------------
        ! Error handling
        !-------------------------------------------------
        ERROR(NEW_ACTION)
            TRYNEXT;
    ENDPROC

    LOCAL PROC initializeMainModule()
        !-------------------------------------------------   
        ! Check if:
        ! * IRB14000 is used, i.e. YuMi.
        ! * Synchronization should be used
        !   (if several TCP robots).
        !------------------------------------------------- 
        VAR num listno := 0;
        VAR string name := "";
        VAR bool tcp_rob := FALSE;
        VAR num number_of_tcp_robots := 0;
        
        ! Count the number of TCP robots in the system.
        WHILE GetNextMechUnit(listno, name, \TCPRob:=tcp_rob) DO
            IF tcp_rob THEN
                number_of_tcp_robots := number_of_tcp_robots + 1;
            ENDIF
        ENDWHILE
        
        using_irb14000 := (GetMecUnitName(ROB_ID) = "ROB_L" OR GetMecUnitName(ROB_ID) = "ROB_R");
        use_sync := (number_of_tcp_robots > 1);
        
        !-------------------------------------------------   
        ! Setup a interrupt signal:
        ! * GO_TO_HOME_POSITION - Move the robot to
        !                         the home position.
        !
        ! Add a digital input (DI) signal 
        ! Controller tab -> Configuration Editor ->
        ! I/O System -> Signal
        !-------------------------------------------------   
        IDelete interupt_go_to_home_position;
        CONNECT interupt_go_to_home_position WITH goToHomePositionHandler;
        ISignalDI GO_TO_HOME_POSITION, HIGH, interupt_go_to_home_position; 
        
        !-------------------------------------------------   
        ! Setup a timer that updates a jointtarget with
        ! the current joint values.
        !-------------------------------------------------  
        CONNECT interupt_timer WITH timerHandler;
        ITimer 0.1, interupt_timer;
        
        !-------------------------------------------------
        ! Task actions:
        !-------------------------------------------------
        !
        ! ACTION_IDLE:
        ! Idle until the action is changed.
        !
        ! ACTION_GO_TO_HOME_POSITION:
        ! Move to the defined home position.
        !
        ! ACTION_EGM:
        ! Start to collect reference signals from the 
        ! external EGM server program and go to
        ! idling when finished.
        !
        !------------------------------------------
        ! IMPORTANT: Triggering EGM actions
        !------------------------------------------
        ! Assuming that the relevant signals has
        ! been added to the configuration:
        !
        ! Changing value of signal "EGM_<action>":
        !
        ! * If "EGM_<action>" not added to a list:
        !   Simulation tab -> I/O Simulator -> 
        !   Filter = User List -> Edit Lists -> New List ->
        !   Add "EGM_<action>" to the list -> OK
        !
        ! * Else:
        !   Simulation tab -> I/O Simulator ->
        !   Filter = User List -> Toogle "EGM_<action>" from 0 to 1
        !
        ! The EGM server program must also be up and
        ! running for EGM motion to begin.
        !
        ! Note:
        ! If a Robot Web Servies client is available,
        ! then IO signals can be triggered from an
        ! external source.
        !
        !-------------------------------------------------
        current_action := ACTION_IDLE;
    ENDPROC
    
    LOCAL PROC handleGoToHomePositionAction()
        TPWrite "Action: Go to home postion";
        goToHomePositon;
        current_action := ACTION_IDLE;
    ENDPROC
    
    !-------------------------------------------------
    ! Interupt handlers
    !-------------------------------------------------
    LOCAL TRAP goToHomePositionHandler
        IF current_action = ACTION_IDLE THEN
            current_action := ACTION_GO_TO_HOME_POSITION;
            
            RAISE NEW_ACTION;
        ENDIF
        
        ERROR (NEW_ACTION)
            RAISE NEW_ACTION;
    ENDTRAP
    
    LOCAL TRAP timerHandler
        current_joints := CJointT(); 
    ENDTRAP




    !==================================================================================================
    ! Auxiliary components
    !==================================================================================================    
    !-------------------------------------------------
    ! Auxiliary procedures
    !-------------------------------------------------
    LOCAL PROC goToHomePositon()
        VAR jointtarget joint_position_home;
        VAR speeddata speed := [200, 200, 200, 200];
        
        ! Set home position.
        TEST GetMecUnitName(ROB_ID)
            CASE "ROB_L":
                joint_position_home := [[0, -130, 30, 0, 40, 0], [135, 9E9, 9E9, 9E9, 9E9, 9E9]];
                   
            CASE "ROB_R":
                joint_position_home := [[0, -130, 30, 0, 40, 0], [-135, 9E9, 9E9, 9E9, 9E9, 9E9]];
                
            DEFAULT:
                joint_position_home := [[0, 0, 0, 0, 30, 0], [9E9, 9E9, 9E9, 9E9, 9E9, 9E9]];     
        ENDTEST

        ! Move to home position.
        MoveAbsJ joint_position_home, speed, fine, tool0;
        WaitTime 1;
    ENDPROC

ENDMODULE