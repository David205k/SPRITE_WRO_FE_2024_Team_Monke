def say_hello(name):
    print(f"Hello {name}")

say_hello.__doc__ = 'A simple function that says hello'

print(say_hello.__doc__)
help(say_hello)
