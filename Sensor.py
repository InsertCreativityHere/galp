
class Sensor:
    def __init__(self, name, desc=None, isActive=True, supportsBuffering=True, bufferSize=0, samplingPeriod=1):
        self.name = name;
        self.desc = desc;
        self.isActive = isActive;
        self.supportsBuffering = supportsBuffering;
        self.bufferSize = bufferSize;
        self.samplingPeriod = samplingPeriod;
        self.isBusy = False;
        self.callbackFunction = None;

    def setActive(active):
        self.isActive = active;

#=====  Single Value Sampling =====#
    '''Samples and returns the sensor's current value.
       @returns: The current value of the sensor.'''
    def readValue(self):
        return None;

#===== Batch Value Sampling =====#
    '''Enalbes/Disables value buffering on the sensor. Note, not all sensors support buffering mode.
       Sensors that don't will always return false.
       @param size: The size of the buffer to allocate. If size is larger than the maximum buffer the sensor
                    can support, this throws an exception. If size is 0 or negative, buffering will be disabled.
       @returns: True if the operation succeeded. False otherwise.'''
    def setBufferSize(self, size):
        return False;

    '''Sets how long the sensor should wait between sampling values.
       settings very short periods may cause instabilities in the readings.
       @param period: The delay in seconds to wait between sampling values.'''
    def setSamplingPeriod(self, period):
        self.samplingPeriod = period;
    
    '''Sets the callback function that the sensor will call as data is read during sampling.
       The sensor will call the function every time a sample is taken if buffering is disabled, and it will
       pass in the current sensor value to it. If buffering is enabled, this will wait until either the buffer
       fills, or the batch finishes until calling the function; in this case a list of buffered readings is
       passed to the function.
       @param callback: The function to call as data is read during the batch sampling.'''
    def setCallbackFunction(self, callback):
        self.callbackFunction = callback;

    '''Starts a batch sampling with a specified number of samples to collect. The sensor will continue sampling
       values until sampleCount many have been collected, or the batch is manually stopped.
       @param sampleCount: The number of samplpes to collect before ending the batch. If sampleCount is 0 or
                           negative, the sensor will continue reading samples until manually stopped.
       @param blocking: Whether the function call should block until the batch is completed or not.'''
    def startBatchSample(self, sampleCount, blocking=True):
        pass;

    '''Starts a batch sampling that starts and stops with specified trigger conditions. The sensor will begin
       sampling when the startTrigger condition is satisfied, and will continue sampling until the stopTrigger
       condition is satisfied, or the batch is manually stopped.
       @param startTrigger: Trigger condition that dictates when to start sampling.
       @param stopTrigger: Trigger condition that dictates when to stop sampling.
       @param blocking: Whether the function call should block until the batch is completed or not.'''
    def startBatchSampleWithTriggers(self, startTrigger, stopTrigger, blocking=True):
        pass;

    '''Immediately stops any running batch samples on the sensor. Any blocked calls will return after calling
       this, and any sensor buffers will be flushed.'''
    def stopBatchSample(self):
        pass;
