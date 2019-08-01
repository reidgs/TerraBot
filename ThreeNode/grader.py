


def spawn_grading(g_filename):
    grade_log = open("Log/grader.log", "a+", 0)
    return sp.Popen(["python", "fraduino.py",
            g_filename], stdout = grade_log,
            stderr = grade_log)

def kill_grading(

