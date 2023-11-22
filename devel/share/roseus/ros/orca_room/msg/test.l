;; Auto-generated. Do not edit!


(when (boundp 'orca_room::test)
  (if (not (find-package "ORCA_ROOM"))
    (make-package "ORCA_ROOM"))
  (shadow 'test (find-package "ORCA_ROOM")))
(unless (find-package "ORCA_ROOM::TEST")
  (make-package "ORCA_ROOM::TEST"))

(in-package "ROS")
;;//! \htmlinclude test.msg.html


(defclass orca_room::test
  :super ros::object
  :slots (_x _y _name ))

(defmethod orca_room::test
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:name __name) "")
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _name (string __name))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:serialization-length
   ()
   (+
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; string _name
    4 (length _name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get orca_room::test :md5sum-) "2d9bb10b98c27a155fd867470178e44b")
(setf (get orca_room::test :datatype-) "orca_room/test")
(setf (get orca_room::test :definition-)
      "float64 x
float64 y
string name

")



(provide :orca_room/test "2d9bb10b98c27a155fd867470178e44b")


