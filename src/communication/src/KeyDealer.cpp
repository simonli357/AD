#include "KeyDealer.hpp"
#include <fstream>
#include <sstream>
#include <stdexcept>

using namespace std;

KeyDealer::KeyDealer() : server_public_key(nullptr, EVP_PKEY_free), server_test_public_key(nullptr, EVP_PKEY_free) { load_keys(); }

void KeyDealer::load_keys() {
	server_public_key.reset(load_public_key(serverPubKey));
	server_test_public_key.reset(load_public_key(testServerPubKey));
}

EVP_PKEY *KeyDealer::load_public_key(const string &path) {
	ifstream file(path);
	if (!file) {
		throw runtime_error("Failed to open key file: " + path);
	}

	stringstream buffer;
	buffer << file.rdbuf();
	string key_str = buffer.str();

	BIO *bio = BIO_new_mem_buf(key_str.data(), key_str.size());
	if (!bio) {
		throw runtime_error("BIO creation failed");
	}

	EVP_PKEY *pkey = PEM_read_bio_PUBKEY(bio, nullptr, nullptr, nullptr);
	BIO_free(bio);

	if (!pkey) {
		throw runtime_error("Failed to load public key from: " + path);
	}
	return pkey;
}

vector<unsigned char> KeyDealer::sign_data(EVP_PKEY *pkey, const string &message) {
	if (!pkey) {
		throw invalid_argument("Invalid private key");
	}

	unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> ctx(EVP_MD_CTX_new(), EVP_MD_CTX_free);
	if (!ctx) {
		throw runtime_error("Failed to create signing context");
	}

	if (EVP_DigestSignInit(ctx.get(), nullptr, EVP_sha256(), nullptr, pkey) != 1) {
		throw runtime_error("Signing initialization failed");
	}

	if (EVP_DigestSignUpdate(ctx.get(), message.data(), message.size()) != 1) {
		throw runtime_error("Signing update failed");
	}

	size_t siglen;
	if (EVP_DigestSignFinal(ctx.get(), nullptr, &siglen) != 1) {
		throw runtime_error("Signing length determination failed");
	}

	vector<unsigned char> signature(siglen);
	if (EVP_DigestSignFinal(ctx.get(), signature.data(), &siglen) != 1) {
		throw runtime_error("Signing finalization failed");
	}
	signature.resize(siglen);

	return signature;
}

bool KeyDealer::verify_data(EVP_PKEY *pkey, const string &message, const vector<unsigned char> &signature) {
	if (!pkey) {
		throw invalid_argument("Invalid public key");
	}

	unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> ctx(EVP_MD_CTX_new(), EVP_MD_CTX_free);
	if (!ctx) {
		throw runtime_error("Failed to create verification context");
	}

	if (EVP_DigestVerifyInit(ctx.get(), nullptr, EVP_sha256(), nullptr, pkey) != 1) {
		throw runtime_error("Verification initialization failed");
	}

	if (EVP_DigestVerifyUpdate(ctx.get(), message.data(), message.size()) != 1) {
		throw runtime_error("Verification update failed");
	}

	int result = EVP_DigestVerifyFinal(ctx.get(), signature.data(), signature.size());

	return (result == 1);
}
