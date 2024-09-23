''' 
MIT License

Copyright (c) 2023 Huang I Chun

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''
import argparse
import subprocess
import json

# python client.py -prev longdress_1051.ply -next longdress_1055.ply -out build/output.ply -pos 0.5

parser = argparse.ArgumentParser()
parser.add_argument("-prev", dest="prev", type=str, help="Set previous frame path", required=True)
parser.add_argument("-next", dest="next", type=str, help="Set next frame path", required=True)
parser.add_argument("-out", dest="output", type=str, help="Set output frame path", required=True)
parser.add_argument("-pos", dest="position", type=float, help="Relative position between prev and next frame", required=True)

args = parser.parse_args()

command = [
        "./build/Pipeline",
        f'{args.prev}',
        f'{args.next}',
        f'{args.output}',
        f'{args.position}',
    ]

subprocess.run(command)
