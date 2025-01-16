from standardbots import StandardBotsRobot, models
import cv2 as cv

sdk = StandardBotsRobot(
  url='http://172.29.208.11:3000',
  token='89k148-5kekqk-mnyf6kql1h2o',
  robot_kind=StandardBotsRobot.RobotKind.Live,
)

try:
    # Establish connection
    with sdk.connection():
        # Task #1: Move the Robot
        print("Starting Task #1: Moving the robot.")

        # 1.1 Move the robot using cartesian coordinates
        sdk.movement.brakes.unbrake().ok()
        sdk.movement.position.move(
            position=models.Position(
                unit_kind=models.LinearUnitKind.Meters,
                x=0.5, y=0.0, z=0.3
            ),
            orientation=models.Orientation(
                kind=models.OrientationKindEnum.Quaternion,
                quaternion=models.Quaternion(0.0, 0.0, 0.0, 1.0),
            ),
        ).ok()
        print("Robot moved to specified cartesian coordinates.")

        # 1.2 Move the robot using joint coordinates
        joint_positions = models.JointPositions(
            joints=[0.0, -1.57, 1.57, 0.0, 0.0, 0.0]  # Replace with actual joint values for your robot
        )
        sdk.movement.joints.move(joint_positions).ok()
        print("Robot moved to specified joint positions.")

        # Task #2: Capture an Image
        print("Starting Task #2: Capturing an image with the camera.")

        # Capture an image (Assuming SDK supports image capture)
        image = sdk.camera.capture_image()
        print("Image captured.")

        # Save the image to a file
        image_file = "captured_image.jpg"
        cv.imwrite(image_file, image)
        print(f"Image saved as '{image_file}'.")

        # Optionally display the image (comment out if not needed)
        cv.imshow("Captured Image", image)
        cv.waitKey(0)
        cv.destroyAllWindows()

except Exception as e:
    print("An error occurred:", e)
