#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import urllib.request


class AuboRPC:
    """
    AUBO Sim JSON-RPC communication layer.
    Runs inside AUBO virtual machine.
    """

    def __init__(self, url="http://127.0.0.1:9012/jsonrpc"):
        self.url = url
        self.req_id = 0

    def call(self, method, params=None):
        if params is None:
            params = []

        self.req_id += 1

        payload = {
            "jsonrpc": "2.0",
            "method": method,
            "params": params,
            "id": self.req_id
        }

        data = json.dumps(payload).encode("utf-8")

        request = urllib.request.Request(
            self.url,
            data=data,
            headers={"Content-Type": "application/json"}
        )

        with urllib.request.urlopen(request, timeout=5) as response:
            result = json.loads(response.read().decode("utf-8"))

        if "error" in result:
            raise RuntimeError(result["error"])

        return result.get("result")
