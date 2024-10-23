from pynput import keyboard


def on_press(key):
    try:
        print("Alphanumeric key pressed: {:} ".format(str(key.char)))
    except AttributeError:
        print("special key pressed: {:}".format(str(key)))


def on_release(key):
    print("Key released: {:}".format(str(key)))
    if key == keyboard.Key.esc:
        # Stop listener
        return False


# Collect events until released
with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()