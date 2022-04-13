# eProsima AML-IP docs

This package generates the AML-IP documentation.

> :warning: **TODO**
[Here](https://readthedocs.org/) it can be seen the online documentation hosted in

[readthedocs](https://readthedocs.org/).
This packages is powered by [sphinx](https://www.sphinx-doc.org/en/master/).

---

## Documentation generation

### Dependencies

Before being able to build the documentation, some dependencies need to be installed:

```bash
sudo apt update
sudo apt install -y \
    doxygen \
    python3 \
    python3-pip \
    python3-venv \
    python3-sphinxcontrib.spelling \
    imagemagick
```

### Build documentation

In order to install this package independently, use the following command:

```bash
colcon build --packages-select amlip_docs
```

In order to compile and execute the package **tests**, a specific CMake option is required: `BUILD_DOCUMENTATION_TESTS`.

```bash
colcon build --packages-select amlip_docs --cmake-args -DBUILD_DOCUMENTATION_TESTS
colcon test --packages-select amlip_docs --event-handler console_direct+
```

---

## CMake options

* `BUILD_TESTS`
* `BUILD_DOCUMENTATION_TESTS`

---

## Library documentation

> :warning: **TODO**
