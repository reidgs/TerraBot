from topic_def import sensor_names

#Takes a name and frequency, and outputs the corresponding string message
def tomsg(name, freq):
    if name not in sensor_names:
        print("invalid sensor name %s" % name)
        return None
    return name + '|' + str(freq)
    
#Takes a message, and gives the (name, freq) that made it
def frommsg(msg):
    pair = msg.split('|')
    if len(pair) != 2:
        print("message error")
        exit()
    name = pair[0].strip(' |')
    freq = float(pair[1].strip(' |'))
    return (name, freq)

