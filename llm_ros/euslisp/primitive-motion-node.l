#!/usr/bin/env roseus

(ros::roseus "primitive_motion_node" :anonymous nil)
(ros::load-ros-manifest "std_msgs")
(ros::load-ros-manifest "std_srvs")
(ros::load-ros-manifest "llm_common_msgs")

(defclass primitive-motion-node
  :super propertied-object
  :slots (robot-name
          robot
          robot-interface
          tfl
          primitive-service-name))

(defmethod primitive-motion-node
  (:init  ()
    (send self :set-val 'robot-name (ros::get-param "~robot_name" "pr2"))
    (cond ((string= robot-name "pr2")
              (require :pr2-interface "package://pr2eus/pr2-interface.l")
              (pr2-init)
              (send self :set-val 'robot *pr2*))
            ((string= robot-name "fetch")
              (require :fetch-interface "package://fetcheus/fetch-interface.l")
              (fetch-init)
              (send self :set-val 'robot *fetch*))
            ((string= robot-name "baxter")
              (require :baxter-interface "package://baxtereus/baxter-interface.l")
              (baxter-init)
              (send self :set-val 'robot *baxter*))
            (t (ros::ros-error "unknown robot name ~A" robot-name) (exit 1)))
    (objects (list robot))
    (send self :set-val 'tfl (instance ros::transform-listener :init))
    (send self :set-val 'primitive-service-name (ros::get-param "~primitive_service_name" "~primitive_motion"))
    (ros::advertise-service primitive-service-name llm_common_msgs::PrimitiveQuery #'send self :primitive-motion-service-cb)
    (print (list "primitive-service-name" primitive-service-name))
    self)

  (:primitive-motion-service-cb (req)
    (let ((res (instance llm_common_msgs::PrimitiveQueryResponse :init)))
      (print (send req :primitive)) ;; TODO
      res)))


(defun main ()
  (ros::ros-info "primitive-motion-node initialized")
  (setq *primitive-motion-node* (instance primitive-motion-node :init))
  (ros::spin))

(main)

;; (defvar *namespace* "tabletop_perception")

;; (defun pr2-pregrasp-pose ()
;;   (send *pr2* :reset-manip-pose)
;;   (send *ri* :angle-vector (send *pr2* :angle-vector) 5000)
;;   (send *ri* :wait-interpolation))

;; (defun ros-init ()
;;   (ros::rate 10)
;;   (ros::advertise robot-state-topic-name eus_imitation::Float32MultiArrayStamped 1)
;;   ;; subscribe
;;   (ros::subscribe (format nil "/~A/joy"
;;                           (send self :get-arm-val :larm :controller-frame-id))
;;                   sensor_msgs::Joy #'send self :vive-controller-cb :larm)
;;   (ros::ros-info "Detected left arm controller device ID :  ~A" larm-controller-device-id)
;;   )

;; (defun init-pr2 ()
;;   (pr2-init)
;;   (objects (list *pr2*)))

;; (defun grasp (pos)
;;   (if *grasping-object-p*
;;     (progn
;;       (publish-warn "cancel grasping...")
;;       (send *ri* :stop-motion)
;;       (send *ri* :stop-grasp *arm* :wait t)
;;       (setq *grasping-object-p* nil))
;;     (setq *grasping-object-p* t))
;;   (unless
;;       (send *pr2* *arm* :inverse-kinematics
;;             (make-coords :pos (v+ pos
;;                                   (float-vector -120 0 0))
;;                          :rpy #f(0 0 0))
;;             :rotation-axis t)
;;     (publish-error "out of reach...")
;;     (return-from grasp nil))
;;   (publish-info "openning gripper...")

;;   (setq *grasp-state* "reaching")
;;   (publish-grasp-state)
;;   (publish-info "pre grasp pose...")
;;   (send *ri* :stop-grasp *arm* :wait t)
;;   (send *pr2* :look-at-hand *arm*)
;;   (send *ri* :angle-vector (send *pr2* :angle-vector) 6000)
;;   (send *ri* :wait-interpolation)

;;   (setq *grasp-state* "grasp-approaching")
;;   (publish-grasp-state)
;;   (publish-info "reaching...")
;;   (send *pr2* *arm* :move-end-pos #f(160 0 0) :world)
;;   (send *pr2* :look-at-hand *arm*)
;;   (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
;;   (send *ri* :wait-interpolation)

;;   (setq *grasp-state* "grasp-closing")
;;   (publish-grasp-state)
;;   (publish-info "grasping...")
;;   (let ((grasp-result (send *ri* :start-grasp *arm*)))
;;     (send *ri* :wait-interpolation)
;;     (unless (< 5.0 grasp-result)
;;       (publish-error "failed to grasp ~A" grasp-result)
;;       (send *ri* :stop-grasp *arm* :wait t)
;;       (publish-info "back to pre grasp...")
;;       (pr2-pregrasp-pose)
;;       (return-from grasp nil)))

;;   (setq *grasp-state* "picking")
;;   (publish-grasp-state)
;;   (publish-info "picking up...")
;;   (send *pr2* *arm* :move-end-pos #f(0 0 150) :world)
;;   (send *pr2* :look-at-hand *arm*)
;;   (send *ri* :angle-vector (send *pr2* :angle-vector) 2000)
;;   (send *ri* :wait-interpolation)
;;   (publish-info "grasp succeeded!")

;;   (setq *grasp-state* "placing")
;;   (publish-grasp-state)
;;   (publish-info "placing ...")
;;   (send *pr2* *arm* :move-end-pos #f(0 0 -150) :world)
;;   (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
;;   (send *ri* :wait-interpolation)

;;   (setq *grasp-state* "grasp-opening")
;;   (publish-grasp-state)
;;   (send *ri* :stop-grasp *arm* :wait t)
;;   (publish-info "return object...")

;;   (setq *grasp-state* "leaving")
;;   (publish-grasp-state)
;;   (send *pr2* *arm* :move-end-pos #f(-160 0 0) :world)
;;   (send *ri* :angle-vector (send *pr2* :angle-vector) 3000)
;;   (send *ri* :wait-interpolation)
;;   (publish-info "pre grasp pose")

;;   (setq *grasp-state* "returning")
;;   (publish-grasp-state)
;;   (pr2-pregrasp-pose)
;;   (ros::ros-info "click tabletop object in Rviz or in image_view2!")
;;   (publish-status 'info (format nil "click tabletop object~%in Rviz or in image_view2!"))
;;   )


;; (defun get-ref-coords ()
;;   (let* (bbox-msg boxes box)
;;       (setq bbox-msg (one-shot-subscribe "/tracking_ros/segmentor/output/boxes"
;;                                 jsk_recognition_msgs::BoundingBoxArray
;;                                 :timeout (* 10 1000)))
;;       (setq boxes (send bbox-msg :boxes))
;;       (dolist (b boxes)
;;         (if (eq (send b :label) 0)
;;           (setq box b)))
;;       (setq label (if box (send box :label)))
;;       (send self :set-val 'ref-coords (if box (ros::tf-pose->coords (send box :pose))))
;;       (send self :set-val 'ref-pos (if box (send (send self :get-val 'ref-coords) :worldpos)))
;;       (send self :set-val 'ref-rot (if box (send (send self :get-val 'ref-coords) :worldrot)))
;;       (setq *target-cube* (make-cube (* 1000 (send box :dimensions :x))
;;                                       (* 1000 (send box :dimensions :y))
;;                                       (* 1000 (send box :dimensions :z))))
;;       (send *target-cube* :newcoords (send ref-coords :copy-worldcoords))
;;       (objects (list robot *target-cube*))))

;; (defun main ()
;;   (init-pr2)
;;   (get-ref-coords)
;;   (send *target-cube* :draw))


;; (main)
