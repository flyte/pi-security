# While movement is happening, each loop the value is incremented.
# If no motion is happening, each loop the value is decremented.

while True:
    if input("> ") == "a":
        if counter < 100:
            counter += 1
    else:
        if counter > 0:
            counter -= 1
    print("Counter: %s" % counter)

