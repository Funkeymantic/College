from standardbots import StandardBotsRobot, models
import base64
import time

sdk = StandardBotsRobot(
    url='http://172.29.208.11:3000',
    token='89k148-5kekqk-mnyf6-kql1h2o',
    robot_kind=StandardBotsRobot.RobotKind.Live,
)

body = models.CameraFrameRequest(
    camera_settings=models.CameraSettings(
        brightness=0,
        contrast=50,
        exposure=350,
        sharpness=50,
        hue=0,
        whiteBalance=4600,
        autoWhiteBalance=True,
    )
)

try:
    # Establish connection
    with sdk.connection():
        sdk.movement.brakes.unbrake().ok()
        response = sdk.movement.position.get_arm_position()
        
        # Grabbing position
        try:
            print("starting)")
            arm_rotation = models.ArmJointRotations(joints=(0.0, 0.0, 0.0, 0.0, 1.5708, 0.0))
            joint_positions = models.ArmPositionUpdateRequest(
                kind=models.ArmPositionUpdateRequestKindEnum.JointRotation,
                joint_rotation=arm_rotation
            )

            sdk.movement.position.set_arm_position(joint_positions).ok()
            data = response.ok()
            j0, j1, j2, j3, j4, j5 = data.joint_rotations
            position = data.tooltip_position.position
            orientation = data.tooltip_position.orientation

            print(f'Position: {position}')
            print(f'Orientation: {orientation}')
        except Exception:
            print(response.data.message)

        # Task #1: Move the Robot
        print("Starting Task #1: Moving the robot.")

        # 1.1 Move the robot using cartesian coordinates
        sdk.movement.position.move(
            position=models.Position(
                unit_kind=models.LinearUnitKind.Meters,
                x=-0.755, y=0.2, z=0.7
            ),
            orientation=models.Orientation(
                kind=models.OrientationKindEnum.Quaternion,
                quaternion=models.Quaternion(0.1, 0.0, 1.0, 0.0),
            ),
        ).ok()
        print("Robot moved to specified cartesian coordinates.")

        # 1.2 Move the robot using joint coordinates
        arm_rotation = models.ArmJointRotations(joints=(0.0,-1.5708,1.5708,0.314159,1.5708,0.0))
        joint_positions = models.ArmPositionUpdateRequest(
            kind=models.ArmPositionUpdateRequestKindEnum.JointRotation,
            joint_rotation=arm_rotation
        )
        sdk.movement.position.set_arm_position(joint_positions).ok()
        print("Robot moved to specified joint positions.")

        # Task #2: Capture an Image
        print("Starting Task #2: Capturing an image with the camera.")

        # Capture an image (Assuming SDK supports image capture)
        res = sdk.camera.data.get_color_frame(body)
        print(f'Result Data: {res.data}')
        res.ok()  # Runs okay
        raw_data = res.response.data

        # Extract the base64 encoded data
        base64_data = raw_data.decode().split(",")[1]

        # Decode the base64 data
        image_data = base64.b64decode(base64_data)

        # Write the frame as png
        with open("frame.png", "wb") as f:
            f.write(image_data)

except Exception as e:
    print("An error occurred:", e)
