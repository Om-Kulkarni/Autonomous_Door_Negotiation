import pygame


def run_test():
    """Initializes a joystick and prints all input events to the console."""

    # Basic setup for pygame and the joystick
    pygame.init()
    pygame.joystick.init()

    # Check if any joysticks are connected
    if pygame.joystick.get_count() == 0:
        print("❌ No joystick detected. Please connect your controller.")
        return

    # Initialize the first joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"✅ Joystick '{joystick.get_name()}' detected. Press buttons or move sticks.")
    print("Press Ctrl+C to exit.")

    try:
        # Main loop to listen for events
        while True:
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    print(f"Button Pressed:  {event.button}")

                elif event.type == pygame.JOYBUTTONUP:
                    print(f"Button Released: {event.button}")

                elif event.type == pygame.JOYAXISMOTION:
                    # To avoid spamming, only print axis motion if value is significant
                    if abs(event.value) > 0.1:
                        print(f"Axis Moved:      {event.axis}, Value: {event.value:.3f}")

                elif event.type == pygame.JOYHATMOTION:
                    print(f"D-Pad Moved:     Hat {event.hat}, Value: {event.value}")

    except KeyboardInterrupt:
        print("\n👋 Test finished.")
    finally:
        pygame.quit()


if __name__ == "__main__":
    run_test()
