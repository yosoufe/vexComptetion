


import pygame
import time

# Initialize pygame
pygame.init()

# Initialize the joystick module
pygame.joystick.init()

# Check if there are any joysticks connected
if pygame.joystick.get_count() == 0:
    print("No joysticks connected.")
    pygame.quit()
else:
    # Initialize the first joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Joystick initialized: {joystick.get_name()}")

    try:
        while True:
            # Process any pending events to keep the system responsive
            pygame.event.pump()

            # Read axis values
            x_axis_1 = joystick.get_axis(2)  # Typically, axis 0 is the X-axis
            y_axis_1 = joystick.get_axis(5)  # Typically, axis 1 is the Y-axis
            print(f"Axis Motion 1 - X: {x_axis_1}, Y: {y_axis_1}")

            x_axis_2 = joystick.get_axis(0)  # Typically, axis 0 is the X-axis
            y_axis_2 = joystick.get_axis(1)  # Typically, axis 1 is the Y-axis
            print(f"Axis Motion 2 - X: {x_axis_2}, Y: {y_axis_2}")

            # Read button states
            button_states = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
            print(f"Buttons {button_states}")

            # Optional: Add a small delay to reduce CPU usage
            time.sleep(0.1)

    except KeyboardInterrupt:
        # Quit when the user presses Ctrl+C
        print("Exiting...")
        pygame.quit()
