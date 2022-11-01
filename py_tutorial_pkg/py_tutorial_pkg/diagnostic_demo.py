import rclpy
from rclpy.node import Node
import diagnostic_msgs
import diagnostic_updater
from diagnostic_updater import DiagnosticTask, FunctionDiagnosticTask

class ClassFunction(DiagnosticTask):
    def __init__(self, name="classFunction"):
        DiagnosticTask.__init__(self, name)
        self.__counter = 0

    def run(self, stat):
        self.__counter += 1
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, 'Test is running')
        stat.add('Value', '%f' % self.__counter)
        stat.add('String', 'Toto')
        stat.add('Floating', str(5.55))
        stat.add('Integer', str(5))
        stat.add('Bool', str(True))
        return stat

class DummyClass:
    def produce_diagnostics(self, stat):
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "This is a silly updater.")
        stat.add("Stupidicity of this updater", "1000.")
        return stat

class Minimal(Node):
    def __init__(self):
        super().__init__("minimal_diagnostic")
        self.__counter = 0
        self.__updater = diagnostic_updater.Updater(self)
        self.__updater.setHardwareID("none")
        self.get_logger().info("init minimal diagnostic demo")
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        dc = DummyClass()
        self.__updater.add("Method updater", dc.produce_diagnostics)
        self.__updater.update()
        task = ClassFunction()
        self.__updater.add(task)

        lower = FunctionDiagnosticTask("lower", self.__check_lower)
        upper = FunctionDiagnosticTask("upper", self.__check_upper)
        bound = diagnostic_updater.CompositeDiagnosticTask("bound check")
        bound.addTask(lower)
        bound.addTask(upper)
        self.__updater.add(bound)
        self.__updater.broadcast(diagnostic_msgs.msg.DiagnosticStatus.OK, "-------Doing important initialization stuff.")
        freq_bounds = {'min':0.01, 'max':12} # If you update these values, the
        # HeaderlessTopicDiagnostic will use the new values.
        self.__pub1_freq = diagnostic_updater.HeaderlessTopicDiagnostic("topic1", 
            self.__updater,
            diagnostic_updater.FrequencyStatusParam(freq_bounds, 0.1, 10))
        task = ClassFunction("------high-------")
        self.__pub1_freq.addTask(task)

    def __check_upper(self, stat):
        if self.__counter < 10:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Upper-bound OK")
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Upper Too high")
        return stat

    def __check_lower(self, stat):
        if self.__counter > 5:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "lower-bound OK")
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "lower Too low")
        return stat

    def timer_callback(self):
        self.get_logger().info(f'timer call')
        self.__counter += 1
        self.__pub1_freq.tick()
        if self.__counter == 10:
            self.__updater.removeByName("Method updater")
        

def main():
    rclpy.init()
    node = Minimal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()