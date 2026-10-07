package com.robodog.discovery

import android.content.Context
import android.net.nsd.NsdManager
import android.net.nsd.NsdServiceInfo
import expo.modules.kotlin.modules.Module
import expo.modules.kotlin.modules.ModuleDefinition
import java.nio.charset.StandardCharsets

class RobotDiscoveryModule : Module() {
  private var nsdManager: NsdManager? = null
  private var discoveryListener: NsdManager.DiscoveryListener? = null

  override fun definition() = ModuleDefinition {
    Name("RobotDiscovery")
    Events("onService")

    Function("startDiscovery") { serviceType: String ->
      startDiscovery(serviceType)
    }

    Function("stopDiscovery") {
      stopDiscovery()
    }

    OnDestroy {
      stopDiscovery()
    }
  }

  private fun startDiscovery(serviceType: String) {
    stopDiscovery()
    val context = appContext.reactContext ?: return
    val manager = context.getSystemService(Context.NSD_SERVICE) as? NsdManager ?: return
    val normalizedType = if (serviceType.endsWith(".")) serviceType else "$serviceType."
    val listener = object : NsdManager.DiscoveryListener {
      override fun onDiscoveryStarted(regType: String) = Unit
      override fun onDiscoveryStopped(serviceType: String) = Unit
      override fun onStartDiscoveryFailed(serviceType: String, errorCode: Int) {
        stopDiscovery()
      }
      override fun onStopDiscoveryFailed(serviceType: String, errorCode: Int) {
        stopDiscovery()
      }
      override fun onServiceLost(serviceInfo: NsdServiceInfo) = Unit
      override fun onServiceFound(serviceInfo: NsdServiceInfo) {
        if (serviceInfo.serviceType != normalizedType) return
        resolveService(manager, serviceInfo)
      }
    }
    nsdManager = manager
    discoveryListener = listener
    manager.discoverServices(normalizedType, NsdManager.PROTOCOL_DNS_SD, listener)
  }

  private fun resolveService(manager: NsdManager, serviceInfo: NsdServiceInfo) {
    manager.resolveService(serviceInfo, object : NsdManager.ResolveListener {
      override fun onResolveFailed(serviceInfo: NsdServiceInfo, errorCode: Int) = Unit

      override fun onServiceResolved(resolved: NsdServiceInfo) {
        val host = resolved.host?.hostAddress ?: return
        sendEvent("onService", mapOf(
          "name" to resolved.serviceName,
          "type" to resolved.serviceType,
          "host" to host,
          "port" to resolved.port,
          "txt" to resolved.attributes.mapValues { entry ->
            String(entry.value, StandardCharsets.UTF_8)
          }
        ))
      }
    })
  }

  private fun stopDiscovery() {
    val manager = nsdManager
    val listener = discoveryListener
    discoveryListener = null
    nsdManager = null
    if (manager != null && listener != null) {
      try {
        manager.stopServiceDiscovery(listener)
      } catch (_: IllegalArgumentException) {
      }
    }
  }
}
