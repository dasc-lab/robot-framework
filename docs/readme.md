
# Tech Notes

A simple collection of technical notes that I would rather not forget. 

Built using the brilliant theme `Just the Docs`.

## Reference:
main:
https://pmarsceill.github.io/just-the-docs/

helper:
https://pdmosses.github.io/just-the-docs-tests/
https://pdmosses.github.io/test-nav/docs/Mathjax/

## Usage:

local dev
```
bundle exec jekyll serve
```
and navigate to the listed location. 

After every edit, we can refresh the page to see the updates.

## Development using docker 
from the `robot-framework/docs` directory, run
```
rm -rf Gemfile.lock
docker compose build
docker compose up &
```
and then navigate in a browser to
```
http://0.0.0.0:4000/robot-framework/
```

