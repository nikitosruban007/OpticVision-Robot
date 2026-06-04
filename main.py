import cv2
import numpy as np
import websockets
import asyncio

async def run_robot_line_tracking():
    robot_ip_address = "192.168.1.100"
    websocket_uri = f"ws://{robot_ip_address}/ws"
    video_stream_uri = f"http://{robot_ip_address}:81/stream"

    async with websockets.connect(websocket_uri) as robot_socket:
        await robot_socket.send("speed:150")

        video_capture = cv2.VideoCapture(video_stream_uri)
        is_tracking_active = False

        while True:
            read_successful, original_frame = video_capture.read()
            if not read_successful:
                break

            height, width = original_frame.shape[:2]
            screen_center_x = width // 2

            gray_frame = cv2.cvtColor(original_frame, cv2.COLOR_BGR2GRAY)
            blurred_frame = cv2.GaussianBlur(gray_frame, (5, 5), 0)
            binary_frame = cv2.threshold(blurred_frame, 70, 255, cv2.THRESH_BINARY_INV)[1]
            
            kernel_matrix = np.ones((5, 5), np.uint8)
            cleaned_frame = cv2.morphologyEx(binary_frame, cv2.MORPH_OPEN, kernel_matrix)

            region_of_interest = cleaned_frame[int(height * 0.7):height, 0:width]
            calculated_moments = cv2.moments(region_of_interest)

            if calculated_moments["m00"] > 500:
                is_tracking_active = True
                line_center_x = int(calculated_moments["m10"] / calculated_moments["m00"])

                if line_center_x < screen_center_x - 40:
                    await robot_socket.send("left")
                elif line_center_x > screen_center_x + 40:
                    await robot_socket.send("right")
                else:
                    await robot_socket.send("forward")
            else:
                if is_tracking_active:
                    await robot_socket.send("stop")

            cv2.imshow("Processed Output", cleaned_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                await robot_socket.send("stop")
                break

        video_capture.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    asyncio.run(run_robot_line_tracking())