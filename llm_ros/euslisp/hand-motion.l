#!/usr/bin/env roseus
(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :pr2-imitation-interface "package://eus_imitation/euslisp/pr2-imitation-interface.l")

(setq ik-stop-step 10)
(setq av-tm 800)
(setq av-scale 5.0)
(setq min-time 0.3)

(defun ros-init()
  (ros::roseus "pr2_test_node")
  (setq *tfl* (instance ros::transform-listener :init))
  (ros::advertise "/eus_imitation/robot_action" eus_imitation::Float32MultiArrayStamped 1)
  )

(defun inverse-kinematics (arm target-coords robot &rest args)
  (if (eq arm :arms)
    (send* robot :inverse-kinematics target-coords :rotation-axis (list t t)
          :move-target (list (send robot :larm :end-coords) (send robot :rarm :end-coords))
          :avoid-collision-distance 5 :revert-if-fail nil :stop ik-stop-step
          :debug-view nil args)
    (send* robot :inverse-kinematics target-coords
          :rotation-axis t :move-target (send robot arm :end-coords)
          :avoid-collision-distance 5 :revert-if-fail nil :stop ik-stop-step
          :debug-view nil args)))

(defun get-base->hand-coords (hand-frame-id &key (wait-time 0.1))
  (let ((start-time (ros::time 0)) coords)
    (if (or (null *base-frame-id*) (null hand-frame-id))
      (progn
        (ros::ros-warn "Frame ids are not set properly.")
        (ros::ros-warn "base-frame-id: ~A" *base-frame-id*)
        (ros::ros-warn "hand-frame-id: ~A" hand-frame-id)
        (return-from :get-base->hand-coords nil)))
    (send *tfl* :wait-for-transform *base-frame-id* hand-frame-id start-time wait-time)
    (setq coords (send *tfl* :lookup-transform *base-frame-id* hand-frame-id start-time))
    coords))

(defun main(imitation)
  (pr2-init)
  (send *ri* :angle-vector (send *pr2* :reset-manip-pose) 1000)
  (objects (pr2))

  (ros-init)


  (setq *base-frame-id* "base_footprint")
  (setq *hand-frame-id* "right_hand")

  (while (ros::ok)
    (ros::rate 5)
    (ros::spin-once)
    (setq *hand-coords* (get-base->hand-coords *hand-frame-id*))
    (if *hand-coords*
      (progn
        (inverse-kinematics :larm *hand-coords* *pr2*)
        (send *ri* :angle-vector (send *pr2* :angle-vector)
              av-tm :larm-controller 0
              :min-time min-time :scale av-scale))
      (print "No coords"))
    )
  )

(main nil)
