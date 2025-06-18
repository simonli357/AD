#pragma once

#include <memory>
#include <openssl/err.h>
#include <openssl/evp.h>
#include <openssl/pem.h>
#include <ros/package.h>
#include <string>
#include <vector>

class KeyDealer {
  public:
	KeyDealer();
	KeyDealer(KeyDealer &&) = default;
	KeyDealer(const KeyDealer &) = delete;
	KeyDealer &operator=(KeyDealer &&) = delete;
	KeyDealer &operator=(const KeyDealer &) = delete;
	~KeyDealer() = default;

	std::vector<unsigned char> sign_data(EVP_PKEY *pkey, const std::string &message);
	bool verify_data(EVP_PKEY *pkey, const std::string &message, const std::vector<unsigned char> &signature);

  private:
	const std::string package_path = ros::package::getPath("communication");
	const std::string keyDir = package_path + "/src/keys";

	const std::string testServerPubKey = keyDir + "/publickey_server_test.pem";
	const std::string serverPubKey = keyDir + "/publickey_server.pem";

	std::unique_ptr<EVP_PKEY, decltype(&::EVP_PKEY_free)> server_public_key;
	std::unique_ptr<EVP_PKEY, decltype(&::EVP_PKEY_free)> server_test_public_key;

	void load_keys();
	EVP_PKEY *load_public_key(const std::string &path);
};
;
