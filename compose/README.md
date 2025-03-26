## Generate compose files from custom templates

```bash
pip install -r requirements.txt
```

Modify config or templates and run the generator:

```bash
python3 compose_generator.py
```

### Use hardware accelerated compose templates

```bash
python3 compose_generator.py --nvidia
```

**Change X socket (default: X1)**

```bash
python3 compose_generator.py --nvidia --xsocket X0
```
