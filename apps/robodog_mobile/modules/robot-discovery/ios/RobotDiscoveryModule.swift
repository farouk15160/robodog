import ExpoModulesCore

public final class RobotDiscoveryModule: Module, NetServiceBrowserDelegate, NetServiceDelegate {
  private var browser: NetServiceBrowser?
  private var services: [String: NetService] = [:]

  public func definition() -> ModuleDefinition {
    Name("RobotDiscovery")
    Events("onService")
    Function("startDiscovery") { (serviceType: String) in
      self.stop()
      let browser = NetServiceBrowser()
      browser.delegate = self
      self.browser = browser
      browser.searchForServices(ofType: serviceType, inDomain: "local.")
    }
    Function("stopDiscovery") { self.stop() }
    OnDestroy { self.stop() }
  }

  private func stop() {
    browser?.stop()
    browser = nil
    services.values.forEach { $0.stop() }
    services.removeAll()
  }

  public func netServiceBrowser(_ browser: NetServiceBrowser, didFind service: NetService, moreComing: Bool) {
    services[service.name] = service
    service.delegate = self
    service.resolve(withTimeout: 5)
  }

  public func netServiceDidResolveAddress(_ sender: NetService) {
    guard let host = sender.hostName else { return }
    var txt: [String: String] = [:]
    if let data = sender.txtRecordData() {
      for (key, value) in NetService.dictionary(fromTXTRecord: data) {
        if let value, let text = String(data: value, encoding: .utf8) { txt[key] = text }
      }
    }
    sendEvent("onService", [
      "name": sender.name,
      "type": sender.type,
      "host": host,
      "port": sender.port,
      "txt": txt
    ])
  }
}
