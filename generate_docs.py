Import("env")
import subprocess

def gen_docs(*args, **kwargs):
    return subprocess.call(["doxygen", "Doxyfile"], shell=True)

env.AddPostAction("buildprog", gen_docs)