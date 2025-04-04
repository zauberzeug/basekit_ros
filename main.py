#! /usr/bin/env python
from nicegui import app, ui


def startup() -> None:
    @ui.page('/')
    def home_page() -> None:
        ui.label('Hello to your new project: basekit_ros!').classes('text-4xl absolute-center')


@app.get('/status')
def status() -> dict[str, str]:
    return {'status': 'ok'}


app.on_startup(startup)

ui.run(title='basekit_ros')
