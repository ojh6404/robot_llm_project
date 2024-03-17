#!/usr/bin/env roseus
(ros::load-ros-manifest "roseus")
(require :pr2-interface "package://pr2eus/pr2-interface.l")
(ros::load-ros-manifest "jsk_recognition_msgs")

(setq ik-stop-step 10)
(setq av-tm 800)
(setq av-scale 5.0)
(setq min-time 0.3)
(setq *object-coords* nil)

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

(defun box-cb (msg)
  (let* (boxes box)
    (setq boxes (send msg :boxes))
    (if (not boxes)
      (return-from box-cb))
    (dolist (b boxes)
      (if (eq (send b :label) 0)
        (setq box b)))
    (setq label (if box (send box :label)))
    (setq *object-coords* (if box (ros::tf-pose->coords (send box :pose))))
    (setq *ref-pos* (if box (send *object-coords* :worldpos)))
    (setq *ref-rot* (if box (send *object-coords* :worldrot)))
    (setq *target-cube* (make-cube (* 1000 (send box :dimensions :x))
                                    (* 1000 (send box :dimensions :y))
                                    (* 1000 (send box :dimensions :z))))
    (send *target-cube* :newcoords (send *object-coords* :copy-worldcoords))
    (objects (list *pr2* *target-cube*)))
  )


(defun ros-init()
  (ros::roseus "pr2_test_node")
  (ros::subscribe "/tracking_ros/segmentor/output/boxes" jsk_recognition_msgs::BoundingBoxArray
                  #'box-cb)
  )


(defun main(imitation)
  (pr2-init)
  (send *ri* :angle-vector (send *pr2* :reset-manip-pose) 1000)
  (send *ri* :wait-interpolation)
  (objects (pr2))

  (ros-init)
  (ros::rate 3)
  (setq flag t)

  (while (and (ros::ok) flag)
         (ros::spin-once)
    (if *object-coords*
      (progn
        (send *pr2* :inverse-kinematics *object-coords* :move-target (send *pr2* :larm :end-coords))
        (send *ri* :angle-vector (send *pr2* :angle-vector) 1000 :larm-controller 0)
        (send *ri* :wait-interpolation)
        (setq flag nil)
        )

      (print "No coords"))
         )
  (print "End of main")

  (setq *base-frame-id* "base_footprint")
  (setq *hand-frame-id* "right_hand")

  ;; (while (ros::ok)
  ;;   (ros::rate 3)
  ;;   (ros::spin-once)

  ;;   (if *object-coords*
  ;;     (progn
  ;;       (print "Object coords")
  ;;       (print *object-coords*)
  ;;       ))
  ;;   (if *object-coords*
  ;;     (progn
  ;;       (inverse-kinematics :larm *object-coords* *pr2*)
  ;;       (send *ri* :angle-vector (send *pr2* :angle-vector)
  ;;             av-tm :larm-controller 0
  ;;             :min-time min-time :scale av-scale)
  ;;       (send *ri* :wait-interpolation)
  ;;       )

  ;;     (print "No coords"))

  ;;   )
  )

(main nil)
