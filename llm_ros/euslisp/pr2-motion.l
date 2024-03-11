#!/usr/bin/env roseus
(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :pr2-imitation-interface "package://eus_imitation/euslisp/pr2-imitation-interface.l")

(defun init-pr2 ()
  (pr2-init)
  (objects (list *pr2*)))


(defun get-ref-coords ()
  (let* (bbox-msg boxes box)
      (setq bbox-msg (one-shot-subscribe "/tracking_ros/segmentor/output/boxes"
                                jsk_recognition_msgs::BoundingBoxArray
                                :timeout (* 10 1000)))
      (setq boxes (send bbox-msg :boxes))
      (dolist (b boxes)
        (if (eq (send b :label) 0)
          (setq box b)))
      (setq label (if box (send box :label)))
      (send self :set-val 'ref-coords (if box (ros::tf-pose->coords (send box :pose))))
      (send self :set-val 'ref-pos (if box (send (send self :get-val 'ref-coords) :worldpos)))
      (send self :set-val 'ref-rot (if box (send (send self :get-val 'ref-coords) :worldrot)))
      (setq *target-cube* (make-cube (* 1000 (send box :dimensions :x))
                                      (* 1000 (send box :dimensions :y))
                                      (* 1000 (send box :dimensions :z))))
      (send *target-cube* :newcoords (send ref-coords :copy-worldcoords))
      (objects (list robot *target-cube*))))

(defun main ()
  (init-pr2)
  (get-ref-coords)
  (send *target-cube* :draw))

(defun ros-init ()
  (ros::advertise robot-state-topic-name eus_imitation::Float32MultiArrayStamped 1)
  ;; subscribe
  (ros::subscribe (format nil "/~A/joy"
                          (send self :get-arm-val :larm :controller-frame-id))
                  sensor_msgs::Joy #'send self :vive-controller-cb :larm)
  (ros::ros-info "Detected left arm controller device ID :  ~A" larm-controller-device-id)



  )

(main)
