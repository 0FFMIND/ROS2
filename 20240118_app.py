import time
import redis # 导入redis模块，用于与Redis服务器通信
from flask import Flask # Flask是Python的微型框架，用flask创建应用程序

app = Flask(__name__) # Flask创建了该类的一个实例
cache = redis.Redis(host='redis', port=6379) # 创建一个Redis对象，用于跟Redis服务器交互，其服务器可以通过主机名'redis'访问并侦听默认端口6379

def get_hit_count(): # 定义的retries用于异常处理，而不异常的时候return当前的点击数
    retries = 5
    while True:
        try:
            return cache.incr('hits')
        except redis.exceptions.ConnectionError as exc:
            if retries == 0:
                raise exc
            retries -= 1
            time.sleep(0.5)

@app.route('/') # 定义了Flask应用程序的路由，告诉Flask在访问根URL时执行以下的函数
def hello():
    count = get_hit_count()
    return 'Hello World! I have been seen {} times.\n'.format(count)